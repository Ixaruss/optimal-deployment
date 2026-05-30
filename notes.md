problem 1. logitude degrees are not same and decreases towards poles. our vector data doesnt take this

EPSG:3857 is called Web Mercator. It's the projection used by Google Maps, OpenStreetMap, and basically every web map tile service. So you've definitely seen it — it's why Greenland looks enormous on most web maps.

It's also a metric projection — but with a catch
Yes, it uses meters. But it uses a mathematically simplified version of the Earth — it treats the Earth as a perfect sphere instead of the GRS80 ellipsoid. This makes the math faster and simpler, which is why Google chose it for web tiles. But that simplification introduces distortion.
The distortion is specifically in scale — it gets worse as you move away from the equator. At India's latitudes (8°N to 35°N) the scale error ranges from roughly 1% to 10%. Meaning a cell you think is 30m might actually be 27m or 33m on the ground.


The verdict
For web visualization — EPSG:3857 is fine and industry standard. But for your use case where you specifically need accurate 30m cells across all of India, it's the wrong choice. The sphere approximation is too crude at that resolution.
Stick with EPSG:7755. It was literally designed for India.

=================================================


#include "build.h"

using namespace std;

int main () {
    string input_file_path = "./o-maps/IND_wat_areas/IND_water_areas_dcw.shp";
    cout << EPSG4326_TO_EPSG7755(input_file_path,nullopt,"maps/water-areas/") << endl;
    return 0;
}
