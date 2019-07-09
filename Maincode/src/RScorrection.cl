
int maxU32(int x, int y)
{
return x > y ? x : y;
}

int minU32(int x, int y)
{
return x < y ? x : y;
}

int clipU32(int x, int lower, int upper)
{
return minU32(upper, maxU32(x, lower));
}

unsigned char interpolateU8(float lam, const unsigned char  val1, const unsigned char val2)
{
return convert_uchar8(round( (1.f - lam) * val1 + lam * val2) );
}

void kernel simple_add(global const float* kPrecomputed, global const unsigned char* uncorrectedArray, global unsigned char* correctedArray, const int widthPerFrame, global double* debugger)
{
	/*Each work item processes a line of the image. get_global_id(0) indexes the fast axis (Tiff horizontal) and get_global_id(1) the slow axis (Tiff vertical)*/
	const float kk_float = kPrecomputed[get_global_id(0)];
	const int kk = floor(kk_float);
	const int kk1 = clipU32(kk, 0, widthPerFrame - 1);
	const int kk2 = clipU32(kk + 1, 0, widthPerFrame - 1);
	const unsigned char value1 = uncorrectedArray[get_global_id(1) * widthPerFrame + kk1];
	const unsigned char value2 = uncorrectedArray[get_global_id(1) * widthPerFrame + kk2];
	correctedArray[get_global_id(1) * widthPerFrame + get_global_id(0)] = interpolateU8(kk_float - kk1, value1, value2);
	*debugger = uncorrectedArray[1000];
}