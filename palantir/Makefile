TOPDIR=.

all: Makefile.config
	cd $(TOPDIR)/src && $(MAKE) all
	cd $(TOPDIR)/data && $(MAKE) all
	cd $(TOPDIR)/etc && $(MAKE) all

install:
	cd $(TOPDIR)/src && $(MAKE) install
	cd $(TOPDIR)/data && $(MAKE) install
	cd $(TOPDIR)/etc && $(MAKE) install

clean:
	[ -e Makefile.config ] || touch Makefile.config
	cd $(TOPDIR)/src && $(MAKE) clean
	cd $(TOPDIR)/data && $(MAKE) clean
	cd $(TOPDIR)/etc && $(MAKE) clean
	rm Makefile.config

Makefile.config:
	./configure
