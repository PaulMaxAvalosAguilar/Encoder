#---------------------------------------------------------------------------------
#DO NOT EDIT BELOW THIS LINE
#---------------------------------------------------------------------------------
BUILDDIR    := $(TOP_DIR)/$(BUILDDIRECTORY)/$(VERSION)
TARGETDIR   := $(TOP_DIR)/$(TARGETDIRECTORY)/$(VERSION)
LIBSDIR     := $(TOP_DIR)/$(LIBDIRECTORY)/$(VERSION)
HEADERDIR   := $(TOP_DIR)/$(HEADERSDIRECTORY)

SOURCES     := $(shell find $(SRCDIR) -type f -name *.$(SRCEXT))
OBJECTS     := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(SOURCES:.$(SRCEXT)=.$(OBJEXT)))
DEPENDS     := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(SOURCES:.$(SRCEXT)=.d))
STATLIBS    := $(wildcard $(LIBSDIR)/*.$(SLEXT))
DYNLIBS     := $(wildcard $(LIBSDIR)/*.$(DLEXT))	

#---------------------------------------------------------------------------------

include Makefile.config

#COMPILE PART
$(TARGETDIR)/$(TARGET).$(BINEXT): $(TARGETDIR)/$(TARGET).$(ELFEXT)
	rm -f $@
	$(OBJCOPY) -Obinary $(TARGETDIR)/$(TARGET).$(ELFEXT) $@

$(TARGETDIR)/$(TARGET).$(ELFEXT):  $(OBJECTS) $(STATLIBS) $(DYNLIBS)
	@mkdir -p $(dir $@)
	$(LD) -o $@ $(OBJECTS) $(LDFLAGS)
	$(SIZE) $(TARGETDIR)/$(TARGET).$(ELFEXT)

-include $(DEPENDS)

$(BUILDDIR)/%.$(OBJEXT): $(SRCDIR)/%.$(SRCEXT)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c $< $(INCDEP) -o $@  

