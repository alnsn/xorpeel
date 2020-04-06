# Makefile is very basic and very old-school. It doesn't support
# incremental builds. If unsure, run `make clean` first.
.POSIX:

DSO?=		so  # or dylib
LUAPKG?=	lua # pkg-config name

WARNS?=		-Wall -Wextra
PICFLAGS?=	-fPIC
PICLDFLAGS?=	-fPIC
C99OPTS?=	-std=c99
CXXOPTS?=	-std=c++11 -nostdinc++ -fno-exceptions -fno-rtti

XCFLAGS=	-I. $(CPPFLAGS) $(WARNS) $(C99OPTS)
XCXXFLAGS=	-I. $(CPPFLAGS) $(WARNS) $(CXXOPTS)

OBJ=		graph.o    fastdiv.o
PICO=		graph.pico fastdiv.pico
C_TARGETS=	libxorpeel.$(DSO) # XXX libxorpeel.a libxorpeel_pic.a
LUA_TARGETS=	xorpeel.$(DSO)

.SUFFIXES: .c .cc .o .pico

.cc.o:
	$(CXX) $(XCXXFLAGS) $(CXXFLAGS) -c $< -o $@

.c.o:
	$(CC) $(XCFLAGS) $(CFLAGS)  -c $< -o $@

.cc.pico:
	$(CXX) $(XCXXFLAGS) $(PICFLAGS) $(CXXFLAGS) -c $< -o $@

.c.pico:
	$(CC) $(XCFLAGS) $(PICFLAGS) $(CXXFLAGS) -c $< -o $@

libxorpeel.$(DSO): $(PICO)
	$(CC) $(PICLDFLAGS) $(LDFLAGS) -shared $(PICO) -o $@

all-c: ${C_TARGETS}
all-lua: ${LUA_TARGETS}
all: all-c # XXX all-lua

clean:
	rm -f ${OBJ} ${PICO} ${C_TARGETS} ${LUA_TARGETS}
