bin:
	g++ preprocess/preprocess.cc preprocess/jsonParser.cc preprocess/reproject.cc  preprocess/lib/*.cc -o x $(gdal-config --cflags --libs) -ljsoncpp -lgdal -lpng16 -std=c++23 && ./x

test:
	g++  preprocess/test.cc preprocess/jsonParser.cc preprocess/reproject.cc  preprocess/lib/*.cc -o x $(gdal-config --cflags --libs) -ljsoncpp -lgdal -lpng16 -std=c++23 && ./x

t:
	g++ -O3 -ffast-math -flto -ffp-contract=fast  -march=native common/lineofsight.cc common/mult.cc  common/operations.cc preprocess/jsonParser.cc preprocess/reproject.cc preprocess/lib/*.cc -o x $(gdal-config --cflags --libs) -ljsoncpp -lgdal -lpng16 -std=c++23 && ./x

viewshed:
	g++ -O3 -ffast-math -flto -ffp-contract=fast -fopenmp  -march=native common/lineofsight.cc common/angleviewshed.cc common/mult.cc  common/operations.cc preprocess/jsonParser.cc preprocess/reproject.cc preprocess/lib/*.cc -o x  -ljsoncpp -lgdal -lpng16 -std=c++23 && ./x

coverage:
	g++ -O3 -ffast-math -flto -ffp-contract=fast -fopenmp  -march=native -funroll-loops -fprefetch-loop-arrays -fdata-sections -ffunction-sections -Wl,--gc-sections -Wl,-O2 common/lineofsight.cc common/coverage.cc common/mult.cc  common/operations.cc preprocess/jsonParser.cc preprocess/reproject.cc preprocess/lib/*.cc -o x  -ljsoncpp -lgdal -lpng16 -std=c++23 && ./x

los:
	g++ -O3 -ffast-math -flto -ffp-contract=fast -fopenmp  -march=native common/lineofsight.cc common/radarlos.cc common/mult.cc  common/operations.cc preprocess/jsonParser.cc preprocess/reproject.cc preprocess/lib/*.cc -o x  -ljsoncpp -lgdal -lpng16 -std=c++23 && ./x
