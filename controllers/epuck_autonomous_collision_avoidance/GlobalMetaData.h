#pragma once

#include <AStar.hpp>

// static defined matrix points 
constexpr AStar::Vec2i MP_P1 = { 1,0 }, MP_P2 = { 3,0 },
MP_P3 = { 5,0 }, MP_P4 = { 7,0 },
MP_P5 = { 9,0 }, MP_P6 = { 11,0 },
MP_P7 = { 13,0 }, MP_P8 = { 15,0 },

MP_P9 = { 0,1 }, MP_P10 = { 16,1 },
MP_P11 = { 0,3 }, MP_P12 = { 16,3 },
MP_P13 = { 0,5 }, MP_P14 = { 16,5 },
MP_P15 = { 0,7 }, MP_P16 = { 16,7 },

MP_P17 = { 0,9 }, MP_P18 = { 16,9 },
MP_P19 = { 0,11 }, MP_P20 = { 16,11 },
MP_P21 = { 0,13 }, MP_P22 = { 16,13 },
MP_P23 = { 0,15 }, MP_P24 = { 16,15 },

MP_P25 = { 1,16 }, MP_P26 = { 3,16 },
MP_P27 = { 5,16 }, MP_P28 = { 7,16 },
MP_P29 = { 9,16 }, MP_P30 = { 11,16 },
MP_P31 = { 13,16 }, MP_P32 = { 15,16 };

std::vector<AStar::Vec2i> MP_List = { 
MP_P1, MP_P2, MP_P3, MP_P4, 
MP_P5, MP_P6 , MP_P7, MP_P8, 
MP_P9, MP_P10, MP_P11, MP_P12, 
MP_P13, MP_P14, MP_P15, MP_P16,
MP_P17, MP_P18, MP_P19, MP_P20,
MP_P21, MP_P22, MP_P23, MP_P24,
MP_P25, MP_P26, MP_P27, MP_P28,
MP_P29, MP_P30, MP_P31, MP_P32};