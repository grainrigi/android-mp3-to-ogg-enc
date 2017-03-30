include Makefile.template


oggenc : oggenc.o Mp3Decoder.o Makefile
	${CXX} oggenc.o Mp3Decoder.o -o oggenc ${CFLAGS} ${CXXFLAGS} -L ./libs/armeabi-v7a -logg -lvorbis

