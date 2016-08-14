
CXX=g++ -std=c++11
INCLUDES=-Iinclude -I../serial/include/ -I../libbt/include/
CXXOPTS=-Wall -g -O2
ifeq "$(ENV)" "DEBUG"
    CXXOPTS += -DDEBUG
endif
CXXFLAGS=$(CXXOPTS) $(INCLUDES)
LDFLAGS=
LDLIBS=
# Where .a file must be created
LIB=lib/libscanner.a

default: all
# Object files
OBJS=src/scanner.o

all: lib $(LIB)

lib:
	mkdir lib

$(LIB): $(LIB)($(OBJS))

clean:
	rm -f lib/libscanner.a src/*.o

%.d: %.cc
	$(CXX) -MM -MP -MF $@ -MT "$(@:.d=.o) $@" $(INCLUDES) $<

ifneq "$(MAKECMDGOALS)" "clean"
 -include $(OBJS:.o=.d)
endif
