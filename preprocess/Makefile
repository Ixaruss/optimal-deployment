run:
	g++ -std=c++23 build.cc jsonParser.cc -o x $(gdal-config --cflags --libs) -ljsoncpp && ./x

main:
	g++ -std=c++23 main.cc jsonParser.cc verify.cc lib/*.cc -o x $(gdal-config --cflags --libs) -ljsoncpp -lgdal && ./x

verify:
	g++ preprocess.cc jsonParser.cc reproject.cc lib/*.cc -o x $(gdal-config --cflags --libs) -ljsoncpp -lgdal -lpng16 -std=c++23 && ./x

test:
	g++  test.cc jsonParser.cc lib/*.cc -o x $(gdal-config --cflags --libs) -ljsoncpp -lgdal -lpng16 -std=c++23 && ./x
