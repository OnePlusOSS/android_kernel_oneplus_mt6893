ccflags-y += -I$(src)			# needed for trace events

obj-y					+= ion/

obj-$(CONFIG_ASHMEM)			+= ashmem.o
obj-$(CONFIG_ANDROID_VSOC)		+= vsoc.o
obj-$(CONFIG_ANDROID_INTF_ALARM_DEV)	+= alarm-dev.o
#ifdef OPLUS_FEATURE_HANS_FREEZE
obj-$(CONFIG_OPLUS_HANS) += hans.o
obj-$(CONFIG_OPLUS_HANS) += hans_netfilter.o
#endif /*OPLUS_FEATURE_HANS_FREEZE*/

#add foreground process opt support
obj-n   += oplus_ion/
