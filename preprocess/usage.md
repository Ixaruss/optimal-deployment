// verify vector matrix
verifyBin<uint8_t>("india_map.bin", VTM::calc_rows(RESOLUTION), VTM::calc_cols(RESOLUTION));

// verify elevation matrix
verifyBin<int16_t>("india_elevation.bin", ELEV_ROWS, ELEV_COLS);

// verify feasible grid
verifyBin<FeasibleCell>("feasible.bin", 1, grid.cells.size());


// write vector matrix
vector<uint8_t> vec_matrix(ROWS * COLS);
writeBin<uint8_t>("india_map.bin", vec_matrix);

// write elevation matrix
vector<int16_t> elev_matrix(ROWS * COLS);
writeBin<int16_t>("elevation_map.bin", elev_matrix);

// read back
vector<uint8_t> vec_in;
readBin<uint8_t>("india_map.bin", vec_in, ROWS * COLS);

vector<int16_t> elev_in;
readBin<int16_t>("elevation_map.bin", elev_in, ROWS * COLS);

// write feasible grid cells
vector<FeasibleCell> cells = grid.cells;
writeBin<FeasibleCell>("feasible.bin", cells);


Bin b;
b.area_op = { 28.5, 77.1,
              28.7, 77.1,
              28.7, 77.4,
              28.5, 77.4 };

FeasibleGrid grid = b.getFeasibleGrid();

// access each feasible cell
for (auto& cell : grid.cells) {
    cout << "r=" << cell.r << " c=" << cell.c << " val=" << (int)cell.value << "\n";
}

// trace back to original matrix anytime
size_t idx = (size_t)grid.cells[0].r * VEC_COLS + grid.cells[0].c;


python3 -c "
from PIL import Image
Image.MAX_IMAGE_PIXELS = None
img = Image.open('proof.png')
img = img.resize((2978, 2786), Image.NEAREST)
img.save('proof_small.png')
print('done')
"
