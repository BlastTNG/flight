TOPDIR = .

all: Makefile.config
	cd $(TOPDIR)/src && $(MAKE) all
	cd $(TOPDIR)/data && $(MAKE) all

install:
	cd $(TOPDIR)/src && $(MAKE) install
	cd $(TOPDIR)/data && $(MAKE) install

clean:
	[ -e Makefile.config ] || touch Makefile.config
	cd $(TOPDIR)/src && $(MAKE) clean
	cd $(TOPDIR)/data && $(MAKE) clean
	rm Makefile.config

Makefile.config:
	./configure
