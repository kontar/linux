#
# Makefile for the TI network device drivers.
#

obj-$(CONFIG_TLAN) += tlan.o
obj-$(CONFIG_CPMAC) += cpmac.o
obj-$(CONFIG_TI_DAVINCI_EMAC) += davinci_emac.o
obj-$(CONFIG_TI_DAVINCI_MDIO) += davinci_mdio.o
obj-$(CONFIG_TI_DAVINCI_CPDMA) += davinci_cpdma.o
obj-$(CONFIG_TI_VIRTUAL_ETH)    += keystone_veth.o
obj-$(CONFIG_TI_KEYSTONE_NET) += keystone_net.o
keystone_net-y += keystone_ethss.o	\
		  keystone_sgmii.o	\
		  keystone_pa.o		\
		  keystone_net_core.o
