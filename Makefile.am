AUTOMAKE_OPTIONS = subdir-objects
ACLOCAL_AMFLAGS = -I config ${ACLOCAL_FLAGS}

# Library File     libbcc_lib_1.0.la
lib_LTLIBRARIES = libbcc_lib.la
# Sources
libbcc_lib_la_SOURCES = bcc_lib.h bcc_lib.c

# Keep Versioning Info
libbcc_lib_la_LDFLAGS = -version-info 1:0:0
# Include Files
bcc_lib_includedir = $(includedir)/bcc_lib
nobase_bcc_lib_include_HEADERS = bcc_lib.h

# bcc tool
bin_PROGRAMS = bcc_tool
bcc_tool_SOURCES = bcc_tool.c
bcc_tool_LDADD = libbcc_lib.la
