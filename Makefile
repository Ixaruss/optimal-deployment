bin:
	g++ preprocess/preprocess.cc preprocess/jsonParser.cc preprocess/reproject.cc  preprocess/lib/*.cc -o x $(gdal-config --cflags --libs) -ljsoncpp -lgdal -lpng16 -std=c++23 && ./x

test:
	g++  preprocess/test.cc preprocess/jsonParser.cc preprocess/reproject.cc  preprocess/lib/*.cc -o x $(gdal-config --cflags --libs) -ljsoncpp -lgdal -lpng16 -std=c++23 && ./x

tool:
	g++ -O3 -ffast-math -flto -ffp-contract=fast -fopenmp  -march=native -funroll-loops -fprefetch-loop-arrays -fdata-sections -ffunction-sections -Wl,--gc-sections -Wl,-O2 cli/main.cc common/lineofsight.cc common/image.cc common/coverage.cc common/angleviewshed.cc common/radarlos.cc common/mult.cc  common/operations.cc preprocess/jsonParser.cc preprocess/reproject.cc preprocess/preprocess.cc preprocess/lib/*.cc -o adapt  -ljsoncpp -lgdal -lpng16 -std=c++23
