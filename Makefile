run:
	g++ -std=c++23 build.cc jsonParser.cc -o x $(gdal-config --cflags --libs) -ljsoncpp && ./x

main:
	g++ -std=c++23 main.cc jsonParser.cc verify.cc lib/*.cc -o x $(gdal-config --cflags --libs) -ljsoncpp -lgdal && ./x

verify:
	g++ preprocess.cc jsonParser.cc reproject.cc lib/*.cc -o x $(gdal-config --cflags --libs) -ljsoncpp -lgdal -lpng16 -std=c++23 && ./x

test:
	g++  preprocess/test.cc preprocess/jsonParser.cc preprocess/reproject.cc  preprocess/lib/*.cc -o x $(gdal-config --cflags --libs) -ljsoncpp -lgdal -lpng16 -std=c++23 && ./x

t:
	g++ -o3 -march=native common/lineofsight.cc common/viewshed.cc common/mult.cc  common/operations.cc preprocess/jsonParser.cc preprocess/reproject.cc preprocess/lib/*.cc -o x $(gdal-config --cflags --libs) -ljsoncpp -lgdal -lpng16 -std=c++23 && ./x
