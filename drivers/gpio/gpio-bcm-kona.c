/*
 * Copyright (C) 2012-2013 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>

#define BCM_GPIO_PASSWD	0x00a5a501
#define GPIO_PER_BANK	32

#define GPIO_BANK(gpio)	((gpio) >> 5)
#define GPIO_BIT(gpio)	((gpio) & (GPIO_PER_BANK - 1))

#define GPIO_GPOR0_OFFSET					0x00000000
#define GPIO_GPIR0_OFFSET					0x00000020
#define GPIO_GPORS0_OFFSET					0x00000040
#define GPIO_GPORC0_OFFSET					0x00000060
#define GPIO_ISR0_OFFSET					0x00000080
#define GPIO_IMR0_OFFSET					0x000000A0
#define GPIO_IMRC0_OFFSET					0x000000C0
#define GPIO_GPCTR0_OFFSET					0x00000100
#define GPIO_GPPLSR0_OFFSET					0x00000500
#define GPIO_GPPWR_OFFSET					0x00000520

#define GPIO_GPCTR0_DBR_SHIFT					5
#define GPIO_GPCTR0_DBR_MASK					0x000001E0

#define GPIO_GPCTR0_ITR_SHIFT					3
#define GPIO_GPCTR0_ITR_MASK					0x00000018
#define GPIO_GPCTR0_ITR_CMD_RISING_EDGE				0x00000001
#define GPIO_GPCTR0_ITR_CMD_FALLING_EDGE			0x00000002
#define GPIO_GPCTR0_ITR_CMD_BOTH_EDGE				0x00000003

#define GPIO_GPCTR0_IOTR_MASK					0x00000001
#define GPIO_GPCTR0_IOTR_CMD_0UTPUT				0x00000000
#define GPIO_GPCTR0_IOTR_CMD_INPUT				0x00000001

#define GPIO_GPCTR0_DB_ENABLE_MASK				0x00000100

#define LOCK_CODE	0xFFFFFFFF
#define UNLOCK_CODE	0x0

struct bcm_kona_gpio_bank {
	int id;
	int irq;
};

struct bcm_kona_gpio {
	struct irq_domain *irq_domain;
	void __iomem *reg_base;
	int num_bank;
	int irq_base;
	spinlock_t lock;
	struct bcm_kona_gpio_bank *banks;
};

static struct gpio_chip bcm_gpio_chip;

static void bcm_kona_gpio_irq_mask(struct irq_data *d);
static void bcm_kona_gpio_irq_unmask(struct irq_data *d);

static struct bcm_kona_gpio bcm_kona_gpio;

static inline u32 bcm_kona_gpio_out_status(int bank)
{
	return GPIO_GPOR0_OFFSET + (bank << 2);
}

static inline u32 bcm_kona_gpio_in_status(int bank)
{
	return GPIO_GPIR0_OFFSET + (bank << 2);
}

static inline u32 bcm_kona_gpio_out_set(int bank)
{
	return GPIO_GPORS0_OFFSET + (bank << 2);
}

static inline u32 bcm_kona_gpio_out_clear(int bank)
{
	return GPIO_GPORC0_OFFSET + (bank << 2);
}

static inline u32 bcm_kona_gpio_int_status(int bank)
{
	return GPIO_ISR0_OFFSET + (bank << 2);
}

static inline u32 bcm_kona_gpio_int_mask(int bank)
{
	return GPIO_IMR0_OFFSET + (bank << 2);
}

static inline u32 bcm_kona_gpio_int_mskclr(int bank)
{
	return GPIO_IMRC0_OFFSET + (bank << 2);
}

static inline u32 bcm_kona_gpio_pwd_status(int bank)
{
	return GPIO_GPPLSR0_OFFSET + (bank << 2);
}

static inline u32 bcm_kona_gpio_control(int gpio)
{
	return GPIO_GPCTR0_OFFSET + (gpio << 2);
}

static int bcm_kona_count_irq_resources(struct platform_device *pdev)
{
	struct resource *res;
	int count = 0;

	for (;;) {
		res = platform_get_resource(pdev, IORESOURCE_IRQ, count);
		if (!res)
			break;
		count++;
	}
	return count;
}

static void  bcm_kona_gpio_set_lockcode_bank(int bank_id, int lockcode)
{
	void __iomem *reg_base = bcm_kona_gpio.reg_base;
	/* This register is password protected  */
	writel(BCM_GPIO_PASSWD, reg_base + GPIO_GPPWR_OFFSET);
	/* Lock the bank */
	writel(lockcode, reg_base + bcm_kona_gpio_pwd_status(bank_id));
}

static inline void bcm_kona_gpio_lock_bank(int bank_id)
{
	bcm_kona_gpio_set_lockcode_bank(bank_id, LOCK_CODE);
}

static inline void bcm_kona_gpio_unlock_bank(int bank_id)
{
	bcm_kona_gpio_set_lockcode_bank(bank_id, UNLOCK_CODE);
}

static void bcm_kona_gpio_set(struct gpio_chip *chip, unsigned gpio,
	int value)
{
	int bank_id = GPIO_BANK(gpio);
	int bit = GPIO_BIT(gpio);
	void __iomem *reg_base = bcm_kona_gpio.reg_base;
	u32 val, reg_offset;
	unsigned long flags;

	spin_lock_irqsave(&bcm_kona_gpio.lock, flags);
	bcm_kona_gpio_unlock_bank(bank_id);

	/* determine the GPIO pin direction */
	val = readl(reg_base + bcm_kona_gpio_control(gpio));
	val &= GPIO_GPCTR0_IOTR_MASK;

	/* this function only applies to output pin */
	if (GPIO_GPCTR0_IOTR_CMD_INPUT == val)
		return;

	reg_offset = value ? bcm_kona_gpio_out_set(bank_id) :
				bcm_kona_gpio_out_clear(bank_id);

	val = readl(reg_base + reg_offset);
	val |= 1 << bit;
	writel(val, reg_base + reg_offset);

	bcm_kona_gpio_lock_bank(bank_id);
	spin_unlock_irqrestore(&bcm_kona_gpio.lock, flags);
}

static int bcm_kona_gpio_get(struct gpio_chip *chip, unsigned gpio)
{
	int bank_id = GPIO_BANK(gpio);
	int bit = GPIO_BIT(gpio);
	void __iomem *reg_base = bcm_kona_gpio.reg_base;
	u32 val, reg_offset;
	unsigned long flags;

	spin_lock_irqsave(&bcm_kona_gpio.lock, flags);
	bcm_kona_gpio_unlock_bank(bank_id);

	/* determine the GPIO pin direction */
	val = readl(reg_base + bcm_kona_gpio_control(gpio));
	val &= GPIO_GPCTR0_IOTR_MASK;

	/* read the GPIO bank status */
	reg_offset = (GPIO_GPCTR0_IOTR_CMD_INPUT == val) ?
					bcm_kona_gpio_in_status(bank_id) :
					bcm_kona_gpio_out_status(bank_id);
	val = readl(reg_base + reg_offset);

	bcm_kona_gpio_lock_bank(bank_id);
	spin_unlock_irqrestore(&bcm_kona_gpio.lock, flags);

	/* return the specified bit status */
	return (val >> bit) & 1;
}

static int bcm_kona_gpio_direction_input(struct gpio_chip *chip,
	unsigned gpio)
{
	void __iomem *reg_base = bcm_kona_gpio.reg_base;
	u32 val;
	unsigned long flags;
	int bank_id = GPIO_BANK(gpio);

	spin_lock_irqsave(&bcm_kona_gpio.lock, flags);
	bcm_kona_gpio_unlock_bank(bank_id);

	val = readl(reg_base + bcm_kona_gpio_control(gpio));
	val &= ~GPIO_GPCTR0_IOTR_MASK;
	val |= GPIO_GPCTR0_IOTR_CMD_INPUT;
	writel(val, reg_base + bcm_kona_gpio_control(gpio));

	bcm_kona_gpio_lock_bank(bank_id);
	spin_unlock_irqrestore(&bcm_kona_gpio.lock, flags);

	return 0;
}

static int bcm_kona_gpio_direction_output(struct gpio_chip *chip,
	unsigned gpio, int value)
{
	void __iomem *reg_base = bcm_kona_gpio.reg_base;
	int bank_id = GPIO_BANK(gpio);
	int bit = GPIO_BIT(gpio);
	u32 val, reg_offset;
	unsigned long flags;

	spin_lock_irqsave(&bcm_kona_gpio.lock, flags);
	bcm_kona_gpio_unlock_bank(bank_id);

	val = readl(reg_base + bcm_kona_gpio_control(gpio));
	val &= ~GPIO_GPCTR0_IOTR_MASK;
	val |= GPIO_GPCTR0_IOTR_CMD_0UTPUT;
	writel(val, reg_base + bcm_kona_gpio_control(gpio));
	reg_offset = value ? bcm_kona_gpio_out_set(bank_id) :
				bcm_kona_gpio_out_clear(bank_id);

	val = readl(reg_base + reg_offset);
	val |= 1 << bit;
	writel(val, reg_base + reg_offset);

	bcm_kona_gpio_lock_bank(bank_id);
	spin_unlock_irqrestore(&bcm_kona_gpio.lock, flags);

	return 0;
}

static int bcm_kona_gpio_to_irq(struct gpio_chip *chip, unsigned gpio)
{
	if (gpio >= bcm_gpio_chip.ngpio)
		return -ENXIO;
	return irq_create_mapping(bcm_kona_gpio.irq_domain, gpio);
}

static int bcm_kona_gpio_set_debounce(struct gpio_chip *chip, unsigned gpio,
	unsigned debounce)
{
	void __iomem *reg_base = bcm_kona_gpio.reg_base;
	u32 val, res;
	unsigned long flags;
	int bank_id = GPIO_BANK(gpio);

	/* debounce must be 1-128ms (or 0) */
	if ((debounce > 0 && debounce < 1000) || debounce > 128000) {
		dev_err(chip->dev, "Debounce value %u not in range\n",
			debounce);
		return -EINVAL;
	}

	/* calculate debounce bit value */
	if (debounce != 0) {
		/* Convert to ms */
		debounce /= 1000;

		/* find the MSB */
		res = fls(debounce) - 1;

		/* Check if MSB-1 is set (round up or down) */
		if (res > 0 && (debounce & 1 << (res - 1)))
			res++;
	}

	/* spin lock for read-modify-write of the GPIO register */
	spin_lock_irqsave(&bcm_kona_gpio.lock, flags);
	bcm_kona_gpio_unlock_bank(bank_id);

	val = readl(reg_base + bcm_kona_gpio_control(gpio));
	val &= ~GPIO_GPCTR0_DBR_MASK;

	if (debounce == 0) {
		/* disable debounce */
		val &= ~GPIO_GPCTR0_DB_ENABLE_MASK;
	} else {
		val |= GPIO_GPCTR0_DB_ENABLE_MASK |
			  (res << GPIO_GPCTR0_DBR_SHIFT);
	}

	writel(val, reg_base + bcm_kona_gpio_control(gpio));

	bcm_kona_gpio_lock_bank(bank_id);
	spin_unlock_irqrestore(&bcm_kona_gpio.lock, flags);

	return 0;
}

static int bcm_kona_gpio_request(struct gpio_chip *chip, unsigned gpio)
{
	struct irq_data d;

	d.hwirq = gpio;
	bcm_kona_gpio_irq_unmask(&d);

	return 0;
}

static void bcm_kona_gpio_free(struct gpio_chip *chip, unsigned gpio)
{
	struct irq_data d;

	d.hwirq = gpio;
	bcm_kona_gpio_irq_mask(&d);
}

static struct gpio_chip bcm_gpio_chip = {
	.label = "bcm-kona-gpio",
	.request = bcm_kona_gpio_request,
	.free = bcm_kona_gpio_free,
	.direction_input = bcm_kona_gpio_direction_input,
	.get = bcm_kona_gpio_get,
	.direction_output = bcm_kona_gpio_direction_output,
	.set = bcm_kona_gpio_set,
	.set_debounce = bcm_kona_gpio_set_debounce,
	.to_irq = bcm_kona_gpio_to_irq,
	.base = 0,
};

static void bcm_kona_gpio_irq_ack(struct irq_data *d)
{
	int gpio = d->hwirq;
	int bank_id = GPIO_BANK(gpio);
	int bit = GPIO_BIT(gpio);
	void __iomem *reg_base = bcm_kona_gpio.reg_base;
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&bcm_kona_gpio.lock, flags);
	bcm_kona_gpio_unlock_bank(bank_id);

	val = readl(reg_base + bcm_kona_gpio_int_status(bank_id));
	val |= 1 << bit;
	writel(val, reg_base + bcm_kona_gpio_int_status(bank_id));

	bcm_kona_gpio_lock_bank(bank_id);
	spin_unlock_irqrestore(&bcm_kona_gpio.lock, flags);
}

static void bcm_kona_gpio_irq_mask(struct irq_data *d)
{
	int gpio = d->hwirq;
	int bank_id = GPIO_BANK(gpio);
	int bit = GPIO_BIT(gpio);
	void __iomem *reg_base = bcm_kona_gpio.reg_base;
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&bcm_kona_gpio.lock, flags);
	bcm_kona_gpio_unlock_bank(bank_id);

	val = readl(reg_base + bcm_kona_gpio_int_mask(bank_id));
	val |= 1 << bit;
	writel(val, reg_base + bcm_kona_gpio_int_mask(bank_id));

	bcm_kona_gpio_lock_bank(bank_id);
	spin_unlock_irqrestore(&bcm_kona_gpio.lock, flags);
}

static void bcm_kona_gpio_irq_unmask(struct irq_data *d)
{
	int gpio = d->hwirq;
	int bank_id = GPIO_BANK(gpio);
	int bit = GPIO_BIT(gpio);
	void __iomem *reg_base = bcm_kona_gpio.reg_base;
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&bcm_kona_gpio.lock, flags);
	bcm_kona_gpio_unlock_bank(bank_id);

	val = readl(reg_base + bcm_kona_gpio_int_mskclr(bank_id));
	val |= 1 << bit;
	writel(val, reg_base + bcm_kona_gpio_int_mskclr(bank_id));

	bcm_kona_gpio_lock_bank(bank_id);
	spin_unlock_irqrestore(&bcm_kona_gpio.lock, flags);
}

static int bcm_kona_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	int gpio = d->hwirq;
	void __iomem *reg_base = bcm_kona_gpio.reg_base;
	u32 lvl_type;
	u32 val;
	unsigned long flags;
	int bank_id = GPIO_BANK(gpio);

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
		lvl_type = GPIO_GPCTR0_ITR_CMD_RISING_EDGE;
		break;

	case IRQ_TYPE_EDGE_FALLING:
		lvl_type = GPIO_GPCTR0_ITR_CMD_FALLING_EDGE;
		break;

	case IRQ_TYPE_EDGE_BOTH:
		lvl_type = GPIO_GPCTR0_ITR_CMD_BOTH_EDGE;
		break;

	case IRQ_TYPE_LEVEL_HIGH:
	case IRQ_TYPE_LEVEL_LOW:
		/* BCM GPIO doesn't support level triggering */
	default:
		dev_err(bcm_gpio_chip.dev, "Invalid BCM GPIO irq type 0x%x\n",
			type);
		return -EINVAL;
	}

	spin_lock_irqsave(&bcm_kona_gpio.lock, flags);
	bcm_kona_gpio_unlock_bank(bank_id);

	val = readl(reg_base + bcm_kona_gpio_control(gpio));
	val &= ~GPIO_GPCTR0_ITR_MASK;
	val |= lvl_type << GPIO_GPCTR0_ITR_SHIFT;
	writel(val, reg_base + bcm_kona_gpio_control(gpio));

	bcm_kona_gpio_lock_bank(bank_id);
	spin_unlock_irqrestore(&bcm_kona_gpio.lock, flags);

	return 0;
}

static void bcm_kona_gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	int bit, bank_id;
	void __iomem *reg_base = bcm_kona_gpio.reg_base;
	unsigned long sta;
	struct bcm_kona_gpio_bank *bank = irq_get_handler_data(irq);
	struct irq_chip *chip = irq_desc_get_chip(desc);

	chained_irq_enter(chip, desc);

	bank_id = bank->id;
	bcm_kona_gpio_unlock_bank(bank_id);

	sta = readl(reg_base + bcm_kona_gpio_int_status(bank_id)) &
	    (~(readl(reg_base + bcm_kona_gpio_int_mask(bank_id))));

	for_each_set_bit(bit, &sta, 32) {
		/*
		 * Clear interrupt before handler is called so we don't
		 * miss any interrupt occurred during executing them.
		 */
		writel(readl(reg_base + bcm_kona_gpio_int_status(bank_id)) |
				(1 << bit),
				reg_base + bcm_kona_gpio_int_status(bank_id));

		/* Invoke interrupt handler */
		generic_handle_irq(gpio_to_irq(GPIO_PER_BANK * bank_id + bit));
	}

	bcm_kona_gpio_lock_bank(bank_id);

	chained_irq_exit(chip, desc);
}

static struct irq_chip bcm_gpio_irq_chip = {
	.name = "bcm-kona-gpio",
	.irq_ack = bcm_kona_gpio_irq_ack,
	.irq_mask = bcm_kona_gpio_irq_mask,
	.irq_unmask = bcm_kona_gpio_irq_unmask,
	.irq_set_type = bcm_kona_gpio_irq_set_type,
};

static struct __initconst of_device_id bcm_kona_gpio_of_match[] = {
	{ .compatible = "brcm,kona-gpio" },
	{}
};
MODULE_DEVICE_TABLE(of, kona_gpio_match);

/*
 * This lock class tells lockdep that GPIO irqs are in a different
 * category than their parents, so it won't report false recursion.
 */
static struct lock_class_key gpio_lock_class;

static int bcm_kona_gpio_irq_map(struct irq_domain *d, unsigned int virq,
				irq_hw_number_t hwirq)
{
	irq_set_chip_data(virq, &d->host_data);
	irq_set_lockdep_class(virq, &gpio_lock_class);
	irq_set_chip_and_handler(virq, &bcm_gpio_irq_chip, handle_simple_irq);
	irq_set_nested_thread(virq, 1);
	set_irq_flags(virq, IRQF_VALID);

	return 0;
}

static void bcm_kona_gpio_irq_unmap(struct irq_domain *d, unsigned int virq)
{
	irq_set_chip_and_handler(virq, NULL, NULL);
	irq_set_chip_data(virq, NULL);
}

static struct irq_domain_ops bcm_kona_irq_ops = {
	.map    = bcm_kona_gpio_irq_map,
	.unmap  = bcm_kona_gpio_irq_unmap,
	.xlate  = irq_domain_xlate_twocell,
};

static void bcm_kona_gpio_reset(int num_bank)
{
	int i;
	void __iomem *reg_base = bcm_kona_gpio.reg_base;

	/* disable interrupts and clear status */
	for (i = 0; i < num_bank; i++) {
		bcm_kona_gpio_unlock_bank(i);
		writel(0xFFFFFFFF, reg_base + bcm_kona_gpio_int_mask(i));
		writel(0xFFFFFFFF, reg_base + bcm_kona_gpio_int_status(i));
		bcm_kona_gpio_lock_bank(i);
	}
}

static int bcm_kona_gpio_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct resource *res;
	struct bcm_kona_gpio_bank *bank;
	int ret;
	int i;

	match = of_match_device(bcm_kona_gpio_of_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "Failed to find gpio controller\n");
		return -ENODEV;
	}

	bcm_kona_gpio.num_bank = bcm_kona_count_irq_resources(pdev);
	if (bcm_kona_gpio.num_bank == 0) {
		dev_err(&pdev->dev, "Couldn't determine # GPIO banks\n");
		return -ENOENT;
	}
	bcm_gpio_chip.of_node = pdev->dev.of_node;
	bcm_gpio_chip.dev = &pdev->dev;
	bcm_gpio_chip.ngpio = bcm_kona_gpio.num_bank * GPIO_PER_BANK;
	bcm_kona_gpio.banks = devm_kzalloc(&pdev->dev,
				       bcm_kona_gpio.num_bank *
				       sizeof(*bcm_kona_gpio.banks),
				       GFP_KERNEL);
	if (!bcm_kona_gpio.banks) {
		dev_err(&pdev->dev, "Couldn't allocate bank structure\n");
		return -ENOMEM;
	}

	bcm_kona_gpio.irq_base = irq_alloc_descs(-1, 0, bcm_gpio_chip.ngpio,
						0);
	if (bcm_kona_gpio.irq_base < 0) {
		dev_err(&pdev->dev, "Couldn't allocate IRQ numbers\n");
		return -ENXIO;
	}
	bcm_kona_gpio.irq_domain = irq_domain_add_linear(pdev->dev.of_node,
						bcm_gpio_chip.ngpio,
						&bcm_kona_irq_ops,
						&bcm_kona_gpio);
	if (!bcm_kona_gpio.irq_domain) {
		dev_err(&pdev->dev, "Couldn't allocate IRQ domain\n");
		ret = -ENXIO;
		goto err_irq_descs;
	}
	for (i = 0; i < bcm_kona_gpio.num_bank; i++) {
		bank = &bcm_kona_gpio.banks[i];
		bank->id = i;
		bank->irq = platform_get_irq(pdev, i);
		if (bank->irq < 0) {
			dev_err(&pdev->dev, "Couldn't get IRQ for bank %d", i);
			ret = -ENOENT;
			goto err_irq_domain;
		}
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Missing MEM resource\n");
		ret = -ENXIO;
		goto err_irq_domain;
	}

	bcm_kona_gpio.reg_base = devm_request_and_ioremap(&pdev->dev, res);
	if (!bcm_kona_gpio.reg_base) {
		dev_err(&pdev->dev, "Couldn't ioremap regs\n");
		ret = -ENXIO;
		goto err_irq_domain;
	}
	dev_info(&pdev->dev, "Setting up Kona GPIO at 0x%p (phys %#x)\n",
		bcm_kona_gpio.reg_base, res->start);

	bcm_kona_gpio_reset(bcm_kona_gpio.num_bank);

	ret = gpiochip_add(&bcm_gpio_chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "Couldn't add GPIO chip -- %d\n", ret);
		goto err_irq_domain;
	}
	for (i = 0; i < bcm_gpio_chip.ngpio; i++) {
		int irq = bcm_kona_gpio_to_irq(&bcm_gpio_chip, i);
		bank = &bcm_kona_gpio.banks[GPIO_BANK(i)];
		irq_set_lockdep_class(irq, &gpio_lock_class);
		irq_set_chip_data(irq, bank);
		irq_set_chip_and_handler(irq, &bcm_gpio_irq_chip,
					 handle_simple_irq);
		set_irq_flags(irq, IRQF_VALID);
	}
	for (i = 0; i < bcm_kona_gpio.num_bank; i++) {
		bank = &bcm_kona_gpio.banks[i];

		irq_set_chained_handler(bank->irq, bcm_kona_gpio_irq_handler);
		irq_set_handler_data(bank->irq, bank);
	}

	spin_lock_init(&bcm_kona_gpio.lock);

	return 0;

err_irq_domain:
	irq_domain_remove(bcm_kona_gpio.irq_domain);

err_irq_descs:
	irq_free_descs(bcm_kona_gpio.irq_base, bcm_gpio_chip.ngpio);

	return ret;
}

static int bcm_kona_gpio_remove(struct platform_device *pdev)
{
	int i, ret;
	struct bcm_kona_gpio_bank *bank;

	for (i = 0; i < bcm_gpio_chip.ngpio; i++)
		gpio_free(i);
	ret = gpiochip_remove(&bcm_gpio_chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "Couldn't remove GPIO chip -- %d\n", ret);
		return ret;
	}
	for (i = 0; i < bcm_kona_gpio.num_bank; i++) {
		bank = &bcm_kona_gpio.banks[i];
		irq_set_chained_handler(bank->irq, NULL);
		irq_set_handler_data(bank->irq, NULL);
	}
	irq_domain_remove(bcm_kona_gpio.irq_domain);
	irq_free_descs(bcm_kona_gpio.irq_base, bcm_gpio_chip.ngpio);

	return 0;
}

static struct platform_driver bcm_kona_gpio_driver = {
	.driver = {
		   .name = "bcm-kona-gpio",
		   .owner = THIS_MODULE,
		   .of_match_table = bcm_kona_gpio_of_match,
		   },
	.probe = bcm_kona_gpio_probe,
	.remove = bcm_kona_gpio_remove,
};

module_platform_driver(bcm_kona_gpio_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom Kona GPIO Driver");
MODULE_LICENSE("GPL v2");
