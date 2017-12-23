// SPDX-License-Identifier: GPL-2.0
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <errno.h>
#include <ctype.h>
#include <limits.h>
#include <sys/xattr.h>
#include <assert.h>

/*
 * Original work by Jeff Garzik
 *
 * External file lists, symlink, pipe and fifo support by Thayne Harbaugh
 * Hard link support by Luciano Rocha
 */

#define xstr(s) #s
#define str(s) xstr(s)

static int newcx;
static unsigned int cpio_hdr_size;
static unsigned int offset;
static unsigned int ino = 721;
static time_t default_mtime;

struct file_handler {
	const char *type;
	int (*handler)(const char *line);
};

static void push_string(const char *name)
{
	unsigned int name_len = strlen(name) + 1;

	fputs(name, stdout);
	putchar(0);
	offset += name_len;
}

static void push_pad (void)
{
	while (offset & 3) {
		putchar(0);
		offset++;
	}
}

static void push_string_padded(const char *name)
{
	push_string(name);
	push_pad();
}

struct cpio_header {
	unsigned int ino;
	unsigned int mode;
	uid_t uid;
	gid_t gid;
	unsigned int nlink;
	time_t mtime;
	size_t filesize;
	int devmajor;
	int devminor;
	int rdevmajor;
	int rdevminor;
	size_t namesize;
	size_t xattrsize;
	unsigned int check;
};

static void push_hdr(const struct cpio_header *hdr)
{
	char s[256];

	if (newcx) {
		sprintf(s, "%s%08X%08X%08lX%08lX%08X%016llX%08X"
			   "%016llX%08X%08X%08X%08X%08X%08X",
			"070703",
			hdr->ino,
			hdr->mode,
			(long)hdr->uid,
			(long)hdr->gid,
			hdr->nlink,
			(long long)hdr->mtime,
			0,
			(long long)hdr->filesize,
			hdr->devmajor,
			hdr->devminor,
			hdr->rdevmajor,
			hdr->rdevminor,
			(unsigned int)hdr->namesize,
			(unsigned int)hdr->xattrsize);
	} else {
		sprintf(s, "%s%08X%08X%08lX%08lX%08X%08lX"
			   "%08X%08X%08X%08X%08X%08X%08X",
			"070701",
			hdr->ino,
			hdr->mode,
			(long)hdr->uid,
			(long)hdr->gid,
			hdr->nlink,
			(long)hdr->mtime,
			(unsigned int)hdr->filesize,
			hdr->devmajor,
			hdr->devminor,
			hdr->rdevmajor,
			hdr->rdevminor,
			(unsigned int)hdr->namesize,
			hdr->check);
	}
	fputs(s, stdout);
	assert((offset & 3) == 0);
	offset += cpio_hdr_size;
}

static void cpio_trailer(void)
{
	const char name[] = "TRAILER!!!";
	struct cpio_header hdr = {
		.nlink = 1,
		.namesize = strlen(name)+1,
	};

	push_hdr(&hdr);
	push_string_padded(name);

	while (offset % 512) {
		putchar(0);
		offset++;
	}
}

struct xattr_hdr {
	char c_size[8]; /* total size including c_size field */
	char c_data[];
};
static unsigned int xattr_buflen;
static char xattr_buf[4096];

static void push_xattrs(void)
{
	if (!newcx || !xattr_buflen)
		return;

	if (fwrite(xattr_buf, xattr_buflen, 1, stdout) != 1)
		fprintf(stderr, "writing xattrs failed\n");
	offset += xattr_buflen;
	xattr_buflen = 0;

	push_pad();
}

static int convert_hex_string(const char *hex_str, char *out, size_t out_size)
{
	char buf[3];
	size_t str_len = strlen(hex_str);

	if (str_len % 2 != 0 || str_len / 2 > out_size)
		return 0;

	buf[2] = '\0';
	while (*hex_str != '\0') {
		buf[0] = *hex_str++;
		buf[1] = *hex_str++;
		*out++ = (char)strtol(buf, NULL, 16);
	}

	return str_len / 2;
}

static int collect_xattr(const char *line)
{
	const char *name, *value;
	size_t name_len, value_len;
	char *buf = xattr_buf + xattr_buflen;
	struct xattr_hdr *hdr = (struct xattr_hdr *)buf;
	char *bufend = xattr_buf + sizeof(xattr_buf);
	char *value_buf;
	size_t xattr_entry_size;
	char size_str[sizeof(hdr->c_size) + 1];

	if (!newcx)
		return 0;

	name = line;
	value = strchr(line, '=');
	if (!value) {
		fprintf(stderr, "Unrecognized xattr format '%s'", line);
		return -1;
	}
	name_len = value - name;
	value++;

	/*
	 * For now we support only hex encoded values.
	 * String or base64 can be added later.
	 */
	if (strncmp(value, "0x", 2)) {
		fprintf(stderr,
			"Only hex encoded xattr value is supported '%s'",
			value);
		return -1;
	}

	value += 2;
	value_buf = buf + sizeof(struct xattr_hdr) + name_len + 1;
	value_len = convert_hex_string(value, value_buf, bufend - value_buf);
	if (value_len == 0) {
		fprintf(stderr, "Failed to parse xattr value '%s'", line);
		return -1;
	}
	xattr_entry_size = sizeof(struct xattr_hdr) + name_len + 1 + value_len;

	sprintf(size_str, "%08X", (unsigned int)xattr_entry_size);
	memcpy(hdr->c_size, size_str, sizeof(hdr->c_size));
	memcpy(hdr->c_data, name, name_len);
	hdr->c_data[name_len] = '\0';
	xattr_buflen += xattr_entry_size;

	return 0;
}

static int cpio_mkslink(const char *name, const char *target,
			 unsigned int mode, uid_t uid, gid_t gid)
{
	if (name[0] == '/')
		name++;
	struct cpio_header hdr = {
		.ino = ino++,
		.mode = S_IFLNK | mode,
		.uid = uid,
		.gid = gid,
		.nlink = 1,
		.mtime = default_mtime,
		.filesize = strlen(target)+1,
		.devmajor = 3,
		.devminor = 1,
		.namesize = strlen(name)+1,
		.xattrsize = xattr_buflen,
	};
	push_hdr(&hdr);
	push_string_padded(name);
	push_xattrs();
	push_string_padded(target);
	return 0;
}

static int cpio_mkslink_line(const char *line)
{
	char name[PATH_MAX + 1];
	char target[PATH_MAX + 1];
	unsigned int mode;
	int uid;
	int gid;
	int rc = -1;

	if (5 != sscanf(line, "%" str(PATH_MAX) "s %" str(PATH_MAX) "s %o %d %d", name, target, &mode, &uid, &gid)) {
		fprintf(stderr, "Unrecognized dir format '%s'", line);
		goto fail;
	}
	rc = cpio_mkslink(name, target, mode, uid, gid);
 fail:
	return rc;
}

static int cpio_mkgeneric(const char *name, unsigned int mode,
		       uid_t uid, gid_t gid)
{
	if (name[0] == '/')
		name++;
	struct cpio_header hdr = {
		.ino = ino++,
		.mode = mode,
		.uid = uid,
		.gid = gid,
		.nlink = 2,
		.mtime = default_mtime,
		.devmajor = 3,
		.devminor = 1,
		.namesize = strlen(name)+1,
		.xattrsize = xattr_buflen,
	};
	push_hdr(&hdr);
	push_string_padded(name);
	push_xattrs();
	return 0;
}

enum generic_types {
	GT_DIR,
	GT_PIPE,
	GT_SOCK
};

struct generic_type {
	const char *type;
	mode_t mode;
};

static struct generic_type generic_type_table[] = {
	[GT_DIR] = {
		.type = "dir",
		.mode = S_IFDIR
	},
	[GT_PIPE] = {
		.type = "pipe",
		.mode = S_IFIFO
	},
	[GT_SOCK] = {
		.type = "sock",
		.mode = S_IFSOCK
	}
};

static int cpio_mkgeneric_line(const char *line, enum generic_types gt)
{
	char name[PATH_MAX + 1];
	unsigned int mode;
	int uid;
	int gid;
	int rc = -1;

	if (4 != sscanf(line, "%" str(PATH_MAX) "s %o %d %d", name, &mode, &uid, &gid)) {
		fprintf(stderr, "Unrecognized %s format '%s'",
			line, generic_type_table[gt].type);
		goto fail;
	}
	mode |= generic_type_table[gt].mode;
	rc = cpio_mkgeneric(name, mode, uid, gid);
 fail:
	return rc;
}

static int cpio_mkdir_line(const char *line)
{
	return cpio_mkgeneric_line(line, GT_DIR);
}

static int cpio_mkpipe_line(const char *line)
{
	return cpio_mkgeneric_line(line, GT_PIPE);
}

static int cpio_mksock_line(const char *line)
{
	return cpio_mkgeneric_line(line, GT_SOCK);
}

static int cpio_mknod(const char *name, unsigned int mode,
		       uid_t uid, gid_t gid, char dev_type,
		       unsigned int maj, unsigned int min)
{
	if (dev_type == 'b')
		mode |= S_IFBLK;
	else
		mode |= S_IFCHR;

	if (name[0] == '/')
		name++;
	struct cpio_header hdr = {
		.ino = ino++,
		.mode = mode,
		.uid = uid,
		.gid = gid,
		.nlink = 1,
		.mtime = default_mtime,
		.devmajor = 3,
		.devminor = 1,
		.rdevmajor = maj,
		.rdevminor = min,
		.namesize = strlen(name)+1,
		.xattrsize = xattr_buflen,
	};
	push_hdr(&hdr);
	push_string_padded(name);
	push_xattrs();
	return 0;
}

static int cpio_mknod_line(const char *line)
{
	char name[PATH_MAX + 1];
	unsigned int mode;
	int uid;
	int gid;
	char dev_type;
	unsigned int maj;
	unsigned int min;
	int rc = -1;

	if (7 != sscanf(line, "%" str(PATH_MAX) "s %o %d %d %c %u %u",
			 name, &mode, &uid, &gid, &dev_type, &maj, &min)) {
		fprintf(stderr, "Unrecognized nod format '%s'", line);
		goto fail;
	}
	rc = cpio_mknod(name, mode, uid, gid, dev_type, maj, min);
 fail:
	return rc;
}

static int cpio_mkfile(const char *name, const char *location,
			unsigned int mode, uid_t uid, gid_t gid,
			unsigned int nlinks)
{
	char *filebuf = NULL;
	struct stat buf;
	size_t size;
	int file = -1;
	int retval;
	int rc = -1;
	int namesize;
	unsigned int i;

	mode |= S_IFREG;

	file = open (location, O_RDONLY);
	if (file < 0) {
		fprintf (stderr, "File %s could not be opened for reading\n", location);
		goto error;
	}

	retval = fstat(file, &buf);
	if (retval) {
		fprintf(stderr, "File %s could not be stat()'ed\n", location);
		goto error;
	}

	filebuf = malloc(buf.st_size);
	if (!filebuf) {
		fprintf (stderr, "out of memory\n");
		goto error;
	}

	retval = read (file, filebuf, buf.st_size);
	if (retval < 0) {
		fprintf (stderr, "Can not read %s file\n", location);
		goto error;
	}

	size = 0;
	for (i = 1; i <= nlinks; i++) {
		/* data goes on last link */
		if (i == nlinks) size = buf.st_size;

		if (name[0] == '/')
			name++;
		namesize = strlen(name) + 1;
		struct cpio_header hdr = {
			.ino = ino,
			.mode = mode,
			.uid = uid,
			.gid = gid,
			.nlink = nlinks,
			.mtime = buf.st_mtime,
			.filesize = size,
			.devmajor = 3,
			.devminor = 1,
			.namesize = namesize,
			/* xattrs go on last link */
			.xattrsize = (i == nlinks) ? xattr_buflen : 0,
		};
		push_hdr(&hdr);
		push_string_padded(name);
		if (hdr.xattrsize)
			push_xattrs();

		if (size) {
			if (fwrite(filebuf, size, 1, stdout) != 1) {
				fprintf(stderr, "writing filebuf failed\n");
				goto error;
			}
			offset += size;
			push_pad();
		}

		name += namesize;
	}
	ino++;
	rc = 0;
	
error:
	if (filebuf) free(filebuf);
	if (file >= 0) close(file);
	return rc;
}

static char *cpio_replace_env(char *new_location)
{
	char expanded[PATH_MAX + 1];
	char *start, *end, *var;

	while ((start = strstr(new_location, "${")) &&
	       (end = strchr(start + 2, '}'))) {
		*start = *end = 0;
		var = getenv(start + 2);
		snprintf(expanded, sizeof expanded, "%s%s%s",
			 new_location, var ? var : "", end + 1);
		strcpy(new_location, expanded);
	}

	return new_location;
}

static int cpio_mkfile_line(const char *line)
{
	char name[PATH_MAX + 1];
	char *dname = NULL; /* malloc'ed buffer for hard links */
	char location[PATH_MAX + 1];
	unsigned int mode;
	int uid;
	int gid;
	int nlinks = 1;
	int end = 0, dname_len = 0;
	int rc = -1;

	if (5 > sscanf(line, "%" str(PATH_MAX) "s %" str(PATH_MAX)
				"s %o %d %d %n",
				name, location, &mode, &uid, &gid, &end)) {
		fprintf(stderr, "Unrecognized file format '%s'", line);
		goto fail;
	}
	if (end && isgraph(line[end])) {
		int len;
		int nend;

		dname = malloc(strlen(line));
		if (!dname) {
			fprintf (stderr, "out of memory (%d)\n", dname_len);
			goto fail;
		}

		dname_len = strlen(name) + 1;
		memcpy(dname, name, dname_len);

		do {
			nend = 0;
			if (sscanf(line + end, "%" str(PATH_MAX) "s %n",
					name, &nend) < 1)
				break;
			len = strlen(name) + 1;
			memcpy(dname + dname_len, name, len);
			dname_len += len;
			nlinks++;
			end += nend;
		} while (isgraph(line[end]));
	} else {
		dname = name;
	}
	rc = cpio_mkfile(dname, cpio_replace_env(location),
	                 mode, uid, gid, nlinks);
 fail:
	if (dname_len) free(dname);
	return rc;
}

static void usage(const char *prog)
{
	fprintf(stderr, "Usage:\n"
		"\t%s [-t <timestamp>] [-x] <cpio_list>\n"
		"\n"
		"<cpio_list> is a file containing newline separated entries that\n"
		"describe the files to be included in the initramfs archive:\n"
		"\n"
		"# a comment\n"
		"file <name> <location> <mode> <uid> <gid> [<hard links>]\n"
		"dir <name> <mode> <uid> <gid>\n"
		"nod <name> <mode> <uid> <gid> <dev_type> <maj> <min>\n"
		"slink <name> <target> <mode> <uid> <gid>\n"
		"pipe <name> <mode> <uid> <gid>\n"
		"sock <name> <mode> <uid> <gid>\n"
		"# xattr line is applied to the next non-xattr entry\n"
		"xattr <xattr_name>=<xattr_val>\n"
		"\n"
		"<name>       name of the file/dir/nod/etc in the archive\n"
		"<location>   location of the file in the current filesystem\n"
		"             expands shell variables quoted with ${}\n"
		"<target>     link target\n"
		"<mode>       mode/permissions of the file\n"
		"<uid>        user id (0=root)\n"
		"<gid>        group id (0=root)\n"
		"<dev_type>   device type (b=block, c=character)\n"
		"<maj>        major number of nod\n"
		"<min>        minor number of nod\n"
		"<hard links> space separated list of other links to file\n"
		"<xattr_name> extended attribute name\n"
		"<xattr_val>  hex-encoded extended attribute value\n"
		"\n"
		"example:\n"
		"# A simple initramfs\n"
		"dir /dev 0755 0 0\n"
		"nod /dev/console 0600 0 0 c 5 1\n"
		"dir /root 0700 0 0\n"
		"# set SELinux label 'system_u:object_r:bin_t:s0' for /sbin directory\n"
		"xattr security.selinux=0x73797374656d5f753a6f626a6563745f723a62696e5f743a733000\n"
		"dir /sbin 0755 0 0\n"
		"file /sbin/kinit /usr/src/klibc/kinit/kinit 0755 0 0\n"
		"\n"
		"<timestamp> is time in seconds since Epoch that will be used\n"
		"as mtime for symlinks, special files and directories. The default\n"
		"is to use the current time for these entries.\n",
		prog);
}

struct file_handler file_handler_table[] = {
	{
		.type    = "file",
		.handler = cpio_mkfile_line,
	}, {
		.type    = "nod",
		.handler = cpio_mknod_line,
	}, {
		.type    = "dir",
		.handler = cpio_mkdir_line,
	}, {
		.type    = "slink",
		.handler = cpio_mkslink_line,
	}, {
		.type    = "pipe",
		.handler = cpio_mkpipe_line,
	}, {
		.type    = "sock",
		.handler = cpio_mksock_line,
	}, {
		.type    = "xattr",
		.handler = collect_xattr,
	}, {
		.type    = NULL,
		.handler = NULL,
	}
};

#define LINE_SIZE (2 * PATH_MAX + 50)

int main (int argc, char *argv[])
{
	FILE *cpio_list;
	char line[LINE_SIZE];
	char *args, *type;
	int ec = 0;
	int line_nr = 0;
	const char *filename;

	default_mtime = time(NULL);
	while (1) {
		int opt = getopt(argc, argv, "t:h:x");
		char *invalid;

		if (opt == -1)
			break;
		switch (opt) {
		case 't':
			default_mtime = strtol(optarg, &invalid, 10);
			if (!*optarg || *invalid) {
				fprintf(stderr, "Invalid timestamp: %s\n",
						optarg);
				usage(argv[0]);
				exit(1);
			}
			break;
		case 'x':
			newcx = 1;
			break;
		case 'h':
		case '?':
			usage(argv[0]);
			exit(opt == 'h' ? 0 : 1);
		}
	}
	cpio_hdr_size = newcx ? 134 : 110;

	if (argc - optind != 1) {
		usage(argv[0]);
		exit(1);
	}
	filename = argv[optind];
	if (!strcmp(filename, "-"))
		cpio_list = stdin;
	else if (!(cpio_list = fopen(filename, "r"))) {
		fprintf(stderr, "ERROR: unable to open '%s': %s\n\n",
			filename, strerror(errno));
		usage(argv[0]);
		exit(1);
	}

	while (fgets(line, LINE_SIZE, cpio_list)) {
		int type_idx;
		size_t slen = strlen(line);

		line_nr++;

		if ('#' == *line) {
			/* comment - skip to next line */
			continue;
		}

		if (! (type = strtok(line, " \t"))) {
			fprintf(stderr,
				"ERROR: incorrect format, could not locate file type line %d: '%s'\n",
				line_nr, line);
			ec = -1;
			break;
		}

		if ('\n' == *type) {
			/* a blank line */
			continue;
		}

		if (slen == strlen(type)) {
			/* must be an empty line */
			continue;
		}

		if (! (args = strtok(NULL, "\n"))) {
			fprintf(stderr,
				"ERROR: incorrect format, newline required line %d: '%s'\n",
				line_nr, line);
			ec = -1;
		}

		for (type_idx = 0; file_handler_table[type_idx].type; type_idx++) {
			int rc;
			if (! strcmp(line, file_handler_table[type_idx].type)) {
				if ((rc = file_handler_table[type_idx].handler(args))) {
					ec = rc;
					fprintf(stderr, " line %d\n", line_nr);
				}
				break;
			}
		}

		if (NULL == file_handler_table[type_idx].type) {
			fprintf(stderr, "unknown file type line %d: '%s'\n",
				line_nr, line);
		}
	}
	if (ec == 0)
		cpio_trailer();

	exit(ec);
}
