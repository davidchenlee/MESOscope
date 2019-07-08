void kernel simple_add(global const float* kPrecomputed, global const unsigned char* uncorrectedArray, global unsigned char* correctedArray, const int widthPerFrame, global double* debugger)
{
	/*Each work item processes a line of the image. get_global_id(1) represents the row number*/
	const float kk_float = kPrecomputed[get_global_id(0)];
	const int kk = floor(kk_float);
	const int kk1 = clipU32(kk, 0, widthPerFrame - 1);
	const int kk2 = clipU32(kk + 1, 0, widthPerFrame - 1);
	cconst unsigned char value2 = uncorrectedArray[get_global_id(1) * widthPerFrame + kk2];
	orrectedArray[get_global_id(1) * widthPerFrame + get_global_id(0)] = interpolateU8(kk_float - kk1, value1, value2);
	*debugger = uncorrectedArray[1000];
}