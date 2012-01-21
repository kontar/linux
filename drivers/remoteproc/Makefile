#
# Generic framework for controlling remote processors
#

obj-$(CONFIG_REMOTEPROC)		+= remoteproc.o
remoteproc-y				:= remoteproc_core.o
remoteproc-$(CONFIG_DEBUG_FS)		+= remoteproc_debugfs.o
remoteproc-$(CONFIG_RPMSG)		+= remoteproc_rpmsg.o
obj-$(CONFIG_OMAP_REMOTEPROC)		+= omap_remoteproc.o
