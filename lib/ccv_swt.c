#include "ccv.h"
#include "ccv_internal.h"

#include <stdio.h>
#include <sys/time.h>
#include <ctype.h>

#define MAX_RAY_LENGTH 20

#include "ipp.h"
#include "ipps.h"

const ccv_swt_param_t ccv_swt_default_params = {
	.interval = 1,
	.same_word_thresh = { 0.1, 0.8 },
	.min_neighbors = 5,
	.scale_invariant = 0,
	.size = 3,
	.low_thresh = 50, //124,
	.high_thresh = 100,
	.max_height = 150, // 300 original
	.min_height = 8,
	.min_area = 38,
	.letter_occlude_thresh = 3,
	.aspect_ratio = 8,
	.std_ratio = 0.83,
	.thickness_ratio = 3.0,
	.height_ratio = 3.0,
	.intensity_thresh = 41,
	.distance_ratio = 2.9,
	.intersect_ratio = 1.9,
	.letter_thresh = 3,
	.elongate_ratio = 1.9,
	.breakdown = 0, // 1 original
	.breakdown_ratio = 1.0,
};

static inline CCV_IMPLEMENT_MEDIAN(_ccv_swt_median, int)

typedef struct {
	int x0, x1, y0, y1;
	int w;
} ccv_swt_stroke_t;




#define less_than(s1, s2, aux) ((s1).w < (s2).w)
static CCV_IMPLEMENT_QSORT(_ccv_swt_stroke_qsort, ccv_swt_stroke_t, less_than)
#undef less_than

// hardcoded 3x3 mask
void ipp_sobel(ccv_dense_matrix_t* a, ccv_dense_matrix_t** b, int type, int dx, int dy)
{
	assert(CCV_GET_CHANNEL(a->type) == CCV_C1);
	ccv_declare_derived_signature(sig, a->sig != 0, ccv_sign_with_format(64, "ipp_sobel(%d,%d)", dx, dy), a->sig, CCV_EOF_SIGN);
	type = (type == 0) ? CCV_32S | CCV_GET_CHANNEL(a->type) : CCV_GET_DATA_TYPE(type) | CCV_GET_CHANNEL(a->type);
	ccv_dense_matrix_t* db = *b = ccv_dense_matrix_renew(*b, a->rows, a->cols, CCV_GET_CHANNEL(a->type) | CCV_ALL_DATA_TYPE, type, sig);
	ccv_object_return_if_cached(, db);

    int size, size1, dx_step, dy_step;
    int width = a->cols;
    int height = a->rows;
    IppiSize roi = {width,height};
    IppStatus sts;

    dx_step = dy_step = 2*width;
    db->step = dx_step;

    sts = ippiFilterSobelNegVertGetBufferSize_8u16s_C1R(roi, ippMskSize3x3, &size);
    sts = ippiFilterSobelHorizGetBufferSize_8u16s_C1R(roi, ippMskSize3x3, &size1); 
    if (size1 > size) size = size1;
    Ipp8u* buffer = (Ipp8u*) malloc(size);

    if (dx) {
        sts = ippiFilterSobelNegVertBorder_8u16s_C1R (a->data.u8, a->step, db->data.s16, dx_step, roi, ippMskSize3x3, ippBorderRepl, 0, buffer);
    } else if (dy) {
        sts = ippiFilterSobelHorizBorder_8u16s_C1R(a->data.u8, a->step, db->data.s16, dy_step, roi, ippMskSize3x3, ippBorderRepl, 0, buffer);
    }
    
    free(buffer);
}
 

void ipp_canny(ccv_dense_matrix_t* a, ccv_dense_matrix_t** b, int type, int size_, double low_thresh, double high_thresh)
{
	assert(CCV_GET_CHANNEL(a->type) == CCV_C1);
	ccv_declare_derived_signature(sig, a->sig != 0, ccv_sign_with_format(64, "ipp_canny(%d,%la,%la)", size_, low_thresh, high_thresh), a->sig, CCV_EOF_SIGN);
	type = (type == 0) ? CCV_8U | CCV_C1 : CCV_GET_DATA_TYPE(type) | CCV_C1;
	ccv_dense_matrix_t* db = *b = ccv_dense_matrix_renew(*b, a->rows, a->cols, CCV_C1 | CCV_ALL_DATA_TYPE, type, sig);
	ccv_object_return_if_cached(, db);

    IppStatus sts;
    int size, srcStep, dx_step, dy_step, dst_step;
    int width = a->cols; int height = a->rows;
    IppiSize roi = {width,height};
    srcStep = dst_step = width;
    dx_step = dy_step = 2*width;

	ccv_dense_matrix_t* dx = 0;
	ipp_sobel(a, &dx, 0, 3, 0);
	ccv_dense_matrix_t* dy = 0;
	ipp_sobel(a, &dy, 0, 0, 3);

    ippiCannyGetSize(roi, &size);
    Ipp8u *buffer = (Ipp8u*) malloc(size);
    sts = ippiCanny_16s8u_C1R(dx->data.s16, dx_step, dy->data.s16, dy_step, db->data.u8, dst_step, roi, low_thresh, high_thresh, buffer);

    ccv_matrix_free(dx);
    ccv_matrix_free(dy);
    free(buffer);
}


/* ccv_swt is only the method to generate stroke width map */
void ccv_swt(ccv_dense_matrix_t* a, ccv_dense_matrix_t** b, int type, ccv_swt_param_t params)
{

	assert(a->type & CCV_C1);
	ccv_declare_derived_signature(sig, a->sig != 0, ccv_sign_with_format(64, "ccv_swt(%d,%d,%d,%d)", params.direction, params.size, params.low_thresh, params.high_thresh), a->sig, CCV_EOF_SIGN);
	type = (type == 0) ? CCV_32S | CCV_C1 : CCV_GET_DATA_TYPE(type) | CCV_C1;
	ccv_dense_matrix_t* db = *b = ccv_dense_matrix_renew(*b, a->rows, a->cols, CCV_C1 | CCV_ALL_DATA_TYPE, type, sig);
	ccv_object_return_if_cached(, db);

	unsigned int elapsed_time;

	elapsed_time = get_current_time();
	ccv_dense_matrix_t* cc = 0;
	ipp_canny(a, &cc, 0, params.size, params.low_thresh, params.high_thresh);
    //printf("canny %ums\n", get_current_time() - elapsed_time);
    //ccv_write(cc, "canny.png", 0, CCV_IO_PNG_FILE, 0);

	elapsed_time = get_current_time();
	ccv_dense_matrix_t* c = 0;
	ccv_close_outline(cc, &c, 0);
	ccv_matrix_free(cc);
    //printf("outline %ums\n", get_current_time() -elapsed_time);
    //ccv_write(c, "4.png", 0, CCV_IO_PNG_FILE, 0);
    
    // sobel precomputed in ipp_canny, so must be 0ms
	elapsed_time = get_current_time();
	ccv_dense_matrix_t* dx = 0;
	ipp_sobel(a, &dx, 0, params.size, 0);
	ccv_dense_matrix_t* dy = 0;
	ipp_sobel(a, &dy, 0, 0, params.size);
    //printf("sobel %ums\n", get_current_time() -elapsed_time);


	int i, j, k, w;
	int* buf = (int*)alloca(sizeof(int) * ccv_max(a->cols, a->rows));
	ccv_array_t* strokes = ccv_array_new(sizeof(ccv_swt_stroke_t), 64, 0);
	unsigned char* b_ptr = db->data.u8;
	unsigned char* c_ptr = c->data.u8;
	unsigned char* dx_ptr = dx->data.u8;
	unsigned char* dy_ptr = dy->data.u8;

	//ccv_zero(db);
	int dx5[] = {-1, 0, 1, 0, 0};
	int dy5[] = {0, 0, 0, -1, 1};
	int dy5_cstep [] = {0, 0, 0, -c->step, c->step};
	int dx9[] = {-1, 0, 1, -1, 0, 1, -1, 0, 1};
	int dy9_dxstep[] = {0, 0, 0, -dx->step, -dx->step, -dx->step, dx->step, dx->step, dx->step};
	int dy9_dystep[] = {0, 0, 0, -dy->step, -dy->step, -dy->step, dy->step, dy->step, dy->step};
	int adx, ady, sx, sy, err, e2, x0, x1, y0, y1, kx, ky;
#define ray_reset() \
	err = adx - ady; e2 = 0; \
	x0 = j; y0 = i;
#define ray_reset_by_stroke(stroke) \
	adx = abs(stroke->x1 - stroke->x0); \
	ady = abs(stroke->y1 - stroke->y0); \
	sx = stroke->x1 > stroke->x0 ? 1 : -1; \
	sy = stroke->y1 > stroke->y0 ? 1 : -1; \
	err = adx - ady; e2 = 0; \
	x0 = stroke->x0; y0 = stroke->y0;
#define ray_increment() \
	e2 = 2 * err; \
	if (e2 > -ady) \
	{ \
		err -= ady; \
		x0 += sx; \
	} \
	if (e2 < adx) \
	{ \
		err += adx; \
		y0 += sy; \
	}
	int rdx, rdy, flag;
#define ray_emit(xx, xy, yx, yy, _for_get_d, _for_set_b, _for_get_b) \
	rdx = _for_get_d(dx_ptr, j, 0) * (xx) + _for_get_d(dy_ptr, j, 0) * (xy); \
	rdy = _for_get_d(dx_ptr, j, 0) * (yx) + _for_get_d(dy_ptr, j, 0) * (yy); \
	adx = abs(rdx); \
	ady = abs(rdy); \
	sx = rdx > 0 ? -params.direction : params.direction; \
	sy = rdy > 0 ? -params.direction : params.direction; \
	/* Bresenham's line algorithm */ \
	ray_reset(); \
	flag = 0; \
	kx = x0; \
	ky = y0; \
	for (w = 0; w < MAX_RAY_LENGTH; w++) \
	{ \
		ray_increment(); \
		if (x0 >= a->cols - 1 || x0 < 1 || y0 >= a->rows - 1 || y0 < 1) \
			break; \
		if (abs(i - y0) >= 2 || abs(j - x0) >= 2) \
		{ /* ideally, I can encounter another edge directly, but in practice, we should search in a small region around it */ \
			int y0_offset = (y0 - i) * c->step; \
            if (c_ptr[x0 + dx5[0] + y0_offset + dy5_cstep[0]] + \
                c_ptr[x0 + dx5[1] + y0_offset + dy5_cstep[1]] + \
                c_ptr[x0 + dx5[2] + y0_offset + dy5_cstep[2]] + \
                c_ptr[x0 + dx5[3] + y0_offset + dy5_cstep[3]] + \
                c_ptr[x0 + dx5[4] + y0_offset + dy5_cstep[4]] \
                == 0) \
            { \
                kx = x0 + dx5[4]; \
                ky = y0 + dy5[4]; \
                continue; \
            } \
            \
			flag = 0; \
			for (k = 0; k < 5; k++) \
			{ \
				kx = x0 + dx5[k]; \
				ky = y0 + dy5[k]; \
				if (c_ptr[kx + y0_offset + dy5_cstep[k]]) \
				{ \
					flag = 1; \
					break; \
				} \
			} \
			if (flag) \
				break; \
		} \
	} \
	if (flag && kx < a->cols - 1 && kx > 0 && ky < a->rows - 1 && ky > 0) \
	{ \
		/* the opposite angle should be in d_p -/+ PI / 6 (otherwise discard),
		 * a faster computation should be:
		 * Tan(d_q - d_p) = (Tan(d_q) - Tan(d_p)) / (1 + Tan(d_q) * Tan(d_p))
		 * and -1 / sqrt(3) < Tan(d_q - d_p) < 1 / sqrt(3)
		 * also, we needs to check the whole 3x3 neighborhood in a hope that we don't miss one or two of them */ \
		flag = 0; \
		int ky_dx_offset = (ky - i) * dx->step; \
		int ky_dy_offset = (ky - i) * dy->step; \
		for (k = 0; k < 9; k++) \
		{ \
			int tn = _for_get_d(dy_ptr, j, 0) * _for_get_d(dx_ptr + ky_dx_offset + dy9_dxstep[k], kx + dx9[k], 0) - \
					 _for_get_d(dx_ptr, j, 0) * _for_get_d(dy_ptr + ky_dy_offset + dy9_dystep[k], kx + dx9[k], 0); \
			int td = _for_get_d(dx_ptr, j, 0) * _for_get_d(dx_ptr + ky_dx_offset + dy9_dxstep[k], kx + dx9[k], 0) + \
					 _for_get_d(dy_ptr, j, 0) * _for_get_d(dy_ptr + ky_dy_offset + dy9_dystep[k], kx + dx9[k], 0); \
			if (tn * 7 < -td * 4 && tn * 7 > td * 4) \
			{ \
				flag = 1; \
				break; \
			} \
		} \
		if (flag) \
		{ \
			x1 = x0; y1 = y0; \
			ray_reset(); \
			w = (int)(sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0)) + 0.5); \
			/* extend the line to be width of 1 */ \
			for (;;) \
			{ \
				if (_for_get_b(b_ptr + (y0 - i) * db->step, x0, 0) == 0 || _for_get_b(b_ptr + (y0 - i) * db->step, x0, 0) > w) \
					_for_set_b(b_ptr + (y0 - i) * db->step, x0, w, 0); \
				if (x0 == x1 && y0 == y1) \
					break; \
				ray_increment(); \
			} \
			ccv_swt_stroke_t stroke = { \
				.x0 = j, \
				.x1 = x1, \
				.y0 = i, \
				.y1 = y1, \
				.w = w \
			}; \
			ccv_array_push(strokes, &stroke); \
		} \
	}
#define for_block(_for_get_d, _for_set_b, _for_get_b) \
	unsigned int elapsed_time = get_current_time();\
	for (i = 0; i < a->rows; i++) \
	{ \
		for (j = 0; j < a->cols; j++) \
			if (c_ptr[j]) \
			{ \
				ray_emit(1, 0, 0, 1, _for_get_d, _for_set_b, _for_get_b); \
				ray_emit(1, -1, 1, 1, _for_get_d, _for_set_b, _for_get_b); \
				ray_emit(1, 1, -1, 1, _for_get_d, _for_set_b, _for_get_b); \
			} \
		b_ptr += db->step; \
		c_ptr += c->step; \
		dx_ptr += dx->step; \
		dy_ptr += dy->step; \
	} \
	elapsed_time = get_current_time() - elapsed_time;\
    /*printf("rays %ums\n", elapsed_time);*/\
	elapsed_time = get_current_time();\
	b_ptr = db->data.u8; \
	/* compute median width of stroke, from shortest strokes to longest */ \
	_ccv_swt_stroke_qsort((ccv_swt_stroke_t*)ccv_array_get(strokes, 0), strokes->rnum, 0); \
	for (i = 0; i < strokes->rnum; i++) \
	{ \
		ccv_swt_stroke_t* stroke = (ccv_swt_stroke_t*)ccv_array_get(strokes, i); \
		ray_reset_by_stroke(stroke); \
		int n = 0; \
		for (;;) \
		{ \
			buf[n++] = _for_get_b(b_ptr + y0 * db->step, x0, 0); \
			if (x0 == stroke->x1 && y0 == stroke->y1) \
				break; \
			ray_increment(); \
		} \
		int nw = _ccv_swt_median(buf, 0, n - 1); \
		if (nw != stroke->w) \
		{ \
			ray_reset_by_stroke(stroke); \
			for (;;) \
			{ \
				_for_set_b(b_ptr + y0 * db->step, x0, nw, 0); \
				if (x0 == stroke->x1 && y0 == stroke->y1) \
					break; \
				ray_increment(); \
			} \
		} \
	}\
	elapsed_time = get_current_time() - elapsed_time;\
    //printf("median %d ms\n", elapsed_time);\

    // sobel output really is CCV_16S
	int type2 = CCV_16S | CCV_C1;
	ccv_matrix_getter(type2, ccv_matrix_setter_getter, db->type, for_block);
#undef for_block
#undef ray_emit
#undef ray_reset
#undef ray_increment
    //ccv_write(dx, "dx.png", 0, CCV_IO_PNG_FILE, 0);
	ccv_array_free(strokes);
	ccv_matrix_free(c);
	ccv_matrix_free(dx);
	ccv_matrix_free(dy);


}

ccv_array_t* _ccv_swt_connected_component(ccv_dense_matrix_t* a, int ratio, int min_height, int max_height, int min_area)
{
	int i, j, k;
	int* a_ptr = a->data.i32;
	int dx8[] = {-1, 1, -1, 0, 1, -1, 0, 1};
	int dy8[] = {0, 0, -1, -1, -1, 1, 1, 1};
	int* marker = (int*)cccalloc(sizeof(int) * a->rows * a->cols,1);
	int* m_ptr = marker;
	ccv_point_t* buffer = (ccv_point_t*)ccmalloc(sizeof(ccv_point_t) * a->rows * a->cols);
	ccv_array_t* contours = ccv_array_new(sizeof(ccv_contour_t*), 5, 0);
    //unsigned long t = get_current_time();
	for (i = 0; i < a->rows; i++)
	{
		for (j = 0; j < a->cols; j++)
			if (a_ptr[j] != 0 && !m_ptr[j])
			{
				m_ptr[j] = 1;
				ccv_contour_t* contour = ccv_contour_new(1);
				ccv_point_t* closed = buffer;
				closed->x = j;
				closed->y = i;
				ccv_point_t* open = buffer + 1;
				for (; closed < open; closed++)
				{
					ccv_contour_push(contour, *closed);
					double w = a_ptr[closed->x + (closed->y - i) * a->cols];
					for (k = 0; k < 8; k++)
					{
						int nx = closed->x + dx8[k];
						int ny = closed->y + dy8[k];
						if (nx >= 0 && nx < a->cols && ny >= 0 && ny < a->rows &&
							a_ptr[nx + (ny - i) * a->cols] != 0 &&
							!m_ptr[nx + (ny - i) * a->cols] &&
							(a_ptr[nx + (ny - i) * a->cols] <= ratio * w && a_ptr[nx + (ny - i) * a->cols] * ratio >= w))
						{
							m_ptr[nx + (ny - i) * a->cols] = 1;
							// compute new average w
							w = (w * (int)(open - closed + 1) + a_ptr[nx + (ny - i) * a->cols]) / (double)(open - closed + 2);
							open->x = nx;
							open->y = ny;
							open++;
						}
					}
				}
				if (contour->rect.height < min_height || contour->rect.height > max_height || contour->size < min_area)
					ccv_contour_free(contour);
				else {
					ccv_array_push(contours, &contour);
                    /*printf("\t%d %d %d %d\n", 
                        contour->rect.x,
                        contour->rect.y,
                        contour->rect.width,
                        contour->rect.height
                        );
                    fflush(stdout);*/
                }
			}
		a_ptr += a->cols;
		m_ptr += a->cols;
	}
    //printf("\t%lums\n", get_current_time() - t);
	ccfree(marker);
	ccfree(buffer);
	return contours;
}


static ccv_array_t* _ccv_swt_connected_letters(ccv_dense_matrix_t* a, ccv_dense_matrix_t* swt, ccv_swt_param_t params)
{
	ccv_array_t* contours = _ccv_swt_connected_component(swt, 3, params.min_height, params.max_height, params.min_area);
	ccv_array_t* letters = ccv_array_new(sizeof(ccv_letter_t), 5, 0);
	int i, j, x, y;
	// merge contours that inside other contours
	int* buffer = (int*)ccmalloc(sizeof(int) * swt->rows * swt->cols);
	double aspect_ratio_inv = 1.0 / params.aspect_ratio;
	for (i = 0; i < contours->rnum; i++)
	{
		ccv_contour_t* contour = *(ccv_contour_t**)ccv_array_get(contours, i);
        //printf("%d %d %d %d\n", contour->rect.x, contour->rect.y, contour->rect.width, contour->rect.height);
		assert(contour->rect.height <= params.max_height && contour->rect.height >= params.min_height);
		double ratio = (double)contour->rect.width / (double)contour->rect.height;

        // in russian license plates width cannot be greater than height
        // TODO parametrize it
        if (ratio > 1.5)
		{
			ccv_contour_free(contour);
			continue;
        }


		if (ratio < aspect_ratio_inv || ratio > params.aspect_ratio)
		{
			ccv_contour_free(contour);
			continue;
		}
        //printf("%d %d %d %d\n", contour->rect.x, contour->rect.y, contour->rect.width, contour->rect.height);
		double xc = (double)contour->m10 / contour->size;
		double yc = (double)contour->m01 / contour->size;
		double af = (double)contour->m20 / contour->size - xc * xc;
		double bf = 2 * ((double)contour->m11 / contour->size - xc * yc);
		double cf = (double)contour->m02 / contour->size - yc * yc;
		double delta = sqrt(bf * bf + (af - cf) * (af - cf));
		ratio = sqrt((af + cf + delta) / (af + cf - delta));
		if (ratio < aspect_ratio_inv || ratio > params.aspect_ratio)
		{
			ccv_contour_free(contour);
			continue;
		}
        //printf("%d %d %d %d\n", contour->rect.x, contour->rect.y, contour->rect.width, contour->rect.height);
		double mean = 0;
		for (j = 0; j < contour->size; j++)
		{
			ccv_point_t* point = (ccv_point_t*)ccv_array_get(contour->set, j);
			mean += buffer[j] = swt->data.i32[point->x + point->y * swt->cols];
		}
		mean = mean / contour->size;
		double variance = 0;
		for (j = 0; j < contour->size; j++)
			variance += (mean - buffer[j]) * (mean - buffer[j]);
		variance = variance / contour->size;
		ccv_letter_t letter;
		letter.std = sqrt(variance);
		letter.mean = mean;
		letter.thickness = _ccv_swt_median(buffer, 0, contour->size - 1);
		letter.rect = contour->rect;
		letter.center.x = letter.rect.x + letter.rect.width / 2;
		letter.center.y = letter.rect.y + letter.rect.height / 2;
		letter.intensity = 0;
		letter.contour = contour;
		ccv_array_push(letters, &letter);
	}
	ccv_array_free(contours);
    
    ccfree(buffer);
    buffer = cccalloc(sizeof(int) * swt->rows * swt->cols, 1);

	ccv_array_t* new_letters = ccv_array_new(sizeof(ccv_letter_t), 5, 0);
	for (i = 0; i < letters->rnum; i++)
	{
		ccv_letter_t* letter = (ccv_letter_t*)ccv_array_get(letters, i);
		for (j = 0; j < letter->contour->size; j++)
		{
			ccv_point_t* point = (ccv_point_t*)ccv_array_get(letter->contour->set, j);
			buffer[point->x + point->y * swt->cols] = i + 1;
		}
	}
	// filter out letters that intersects more than 2 other letters
	int* another = params.letter_occlude_thresh ? (int*)alloca(sizeof(int) * params.letter_occlude_thresh) : 0;
	for (i = 0; i < letters->rnum; i++)
	{
		ccv_letter_t* letter = (ccv_letter_t*)ccv_array_get(letters, i);
		if (letter->std > letter->mean * params.std_ratio)
		{
			ccv_contour_free(letter->contour);
			continue;
		}
		int more = 0;
		if (another)
		{
			// one letter cannot occlude with more than params.letter_occlude_thresh other letters
            memset(another, 0, sizeof(int) * params.letter_occlude_thresh);
			for (x = letter->rect.x; x < letter->rect.x + letter->rect.width; x++)
			{
				for (y = letter->rect.y; y < letter->rect.y + letter->rect.height; y++)
				{
					int group = buffer[x + swt->cols * y];
					if (group && group != i + 1)
					{
						more = 1;
						for (j = 0; j < params.letter_occlude_thresh; j++)
							if (!another[j] || another[j] == group)
							{
								another[j] = group;
								more = 0;
								break;
							}
						if (more)
							break;
					}
				}
				if (more)
					break;
			}
		}
		if (more)
		{
			ccv_contour_free(letter->contour);
			continue;
		}
		for (j = 0; j < letter->contour->set->rnum; j++)
		{
			ccv_point_t* point = (ccv_point_t*)ccv_array_get(letter->contour->set, j);
			letter->intensity += a->data.u8[point->x + point->y * a->step];
		}
		letter->intensity /= letter->contour->size;
		ccv_contour_free(letter->contour);
		letter->contour = 0;
		ccv_array_push(new_letters, letter);
        //printf("%d %d %d %d\n", letter->rect.x, letter->rect.y, letter->rect.width, letter->rect.height);
	}
	ccv_array_free(letters);
	ccfree(buffer);
	return new_letters;
}

typedef struct {
	ccv_letter_t* left;
	ccv_letter_t* right;
	int dx;
	int dy;
} ccv_letter_pair_t;

static int _ccv_in_textline(const void* a, const void* b, void* data)
{
	ccv_letter_pair_t* pair1 = (ccv_letter_pair_t*)a;
	ccv_letter_pair_t* pair2 = (ccv_letter_pair_t*)b;
	if (pair1->left == pair2->left || pair1->right == pair2->right)
	{
		int tn = pair1->dy * pair2->dx - pair1->dx * pair2->dy;
		int td = pair1->dx * pair2->dx + pair1->dy * pair2->dy;
		// share the same end, opposite direction
		if (tn * 7 < -td * 4 && tn * 7 > td * 4)
			return 1;
	} else if (pair1->left == pair2->right || pair1->right == pair2->left) {
		int tn = pair1->dy * pair2->dx - pair1->dx * pair2->dy;
		int td = pair1->dx * pair2->dx + pair1->dy * pair2->dy;
		// share the other end, same direction
		if (tn * 7 < td * 4 && tn * 7 > -td * 4)
			return 1;
	}
	return 0;
}

static void _ccv_swt_add_letter(ccv_textline_t* textline, ccv_letter_t* letter)
{
	if (textline->neighbors == 0)
	{
		textline->rect = letter->rect;
		textline->neighbors = 1;
		textline->letters = (ccv_letter_t**)ccmalloc(sizeof(ccv_letter_t*) * textline->neighbors);
		textline->letters[0] = letter;
	} else {
		int i, flag = 0;
		for (i = 0; i < textline->neighbors; i++)
			if (textline->letters[i] == letter)
			{
				flag = 1;
				break;
			}
		if (flag)
			return;
		if (letter->rect.x < textline->rect.x)
		{
			textline->rect.width += textline->rect.x - letter->rect.x;
			textline->rect.x = letter->rect.x;
		}
		if (letter->rect.x + letter->rect.width > textline->rect.x + textline->rect.width)
			textline->rect.width = letter->rect.x + letter->rect.width - textline->rect.x;
		if (letter->rect.y < textline->rect.y)
		{
			textline->rect.height += textline->rect.y - letter->rect.y;
			textline->rect.y = letter->rect.y;
		}
		if (letter->rect.y + letter->rect.height > textline->rect.y + textline->rect.height)
			textline->rect.height = letter->rect.y + letter->rect.height - textline->rect.y;
		textline->neighbors++;
		textline->letters = (ccv_letter_t**)ccrealloc(textline->letters, sizeof(ccv_letter_t*) * textline->neighbors);
		textline->letters[textline->neighbors - 1] = letter;
	}
}

static ccv_array_t* _ccv_swt_merge_textline(ccv_array_t* letters, ccv_swt_param_t params)
{
	int i, j;
	ccv_array_t* pairs = ccv_array_new(sizeof(ccv_letter_pair_t), letters->rnum, 0);
	double thickness_ratio_inv = 1.0 / params.thickness_ratio;
	double height_ratio_inv = 1.0 / params.height_ratio;
	for (i = 0; i < letters->rnum - 1; i++)
	{
		ccv_letter_t* li = (ccv_letter_t*)ccv_array_get(letters, i);
        //printf("%d %d %d %d\n", li->rect.x, li->rect.y, li->rect.width, li->rect.height);
		for (j = i + 1; j < letters->rnum; j++)
		{
			ccv_letter_t* lj = (ccv_letter_t*)ccv_array_get(letters, j);
			double ratio = (double)li->thickness / lj->thickness;
			if (ratio > params.thickness_ratio || ratio < thickness_ratio_inv) {
                    //printf("fuck\n");
                //printf("%f > %f || %f < %f\n", ratio, params.thickness_ratio, ratio, thickness_ratio_inv);
				continue;
            }

			ratio = (double)li->rect.height / lj->rect.height;
			if (ratio > params.height_ratio || ratio < height_ratio_inv) {
                //if ((i==50) && (j == 49 || j == 52)) {printf("fuck1\n");}
				continue;
            }
			if (abs(li->intensity - lj->intensity) > params.intensity_thresh) {
                //if ((i==50) && (j == 49 || j == 52)) {printf("fuck2\n");}
				continue;
            }    
			int dx = li->rect.x - lj->rect.x + (li->rect.width - lj->rect.width) / 2;
			int dy = li->rect.y - lj->rect.y + (li->rect.height - lj->rect.height) / 2;
			if (abs(dx) > params.distance_ratio * ccv_max(li->rect.width, lj->rect.width)) {
                //if ((i==50) && (j == 49 || j == 52)) {printf("fuck3\n");}
				continue;
            }
			
            // oy = total height of pair
            int oy = ccv_min(li->rect.y + li->rect.height, lj->rect.y + lj->rect.height) - ccv_max(li->rect.y, lj->rect.y); 

			if (oy * params.intersect_ratio < ccv_min(li->rect.height, lj->rect.height)) {
                //printf("%f %d\n",
			    //    oy * params.intersect_ratio,
                //    ccv_min(li->rect.height, lj->rect.height));
                //printf("%d %d %d %d\n", li->rect.x, li->rect.y, li->rect.width, li->rect.height);
                //printf("%d %d %d %d\n", lj->rect.x, lj->rect.y, lj->rect.width, lj->rect.height);
				continue;
            }    
            //printf("%d %d %d %d\n", li->rect.x, li->rect.y, li->rect.width, li->rect.height);
            //printf("%d %d %d %d\n", lj->rect.x, lj->rect.y, lj->rect.width, lj->rect.height);
            //printf("%d %d %d %d\n", li->rect.x, li->rect.y, li->rect.width, li->rect.height);
            //printf("%d %d %d %d\n", lj->rect.x, lj->rect.y, lj->rect.width, lj->rect.height);
			ccv_letter_pair_t pair = { .left = li, .right = lj, .dx = dx, .dy = dy };
            //printf("%d %d %d %d\n", li->rect.x, li->rect.y, li->rect.width, li->rect.height);
            //printf("%d %d %d %d\n", lj->rect.x, lj->rect.y, lj->rect.width, lj->rect.height);
			ccv_array_push(pairs, &pair);
		}
	}
	ccv_array_t* idx = 0;
	int nchains = ccv_array_group(pairs, &idx, _ccv_in_textline, 0);
	ccv_textline_t* chain = (ccv_textline_t*)ccmalloc(nchains * sizeof(ccv_textline_t));
	for (i = 0; i < nchains; i++)
		chain[i].neighbors = 0;
	for (i = 0; i < pairs->rnum; i++)
	{
		j = *(int*)ccv_array_get(idx, i);
		_ccv_swt_add_letter(chain + j,((ccv_letter_pair_t*)ccv_array_get(pairs, i))->left);
		_ccv_swt_add_letter(chain + j, ((ccv_letter_pair_t*)ccv_array_get(pairs, i))->right);
		//ccv_letter_t* li = ((ccv_letter_pair_t*)ccv_array_get(pairs, i))->left;
		//ccv_letter_t* lj = ((ccv_letter_pair_t*)ccv_array_get(pairs, i))->right;
	}
	ccv_array_free(idx);
	ccv_array_free(pairs);
	ccv_array_t* regions = ccv_array_new(sizeof(ccv_textline_t), 5, 0);
	for (i = 0; i < nchains; i++)
		if (chain[i].neighbors >= params.letter_thresh && chain[i].rect.width > chain[i].rect.height * params.elongate_ratio) {
            //printf("%d %d %d %d\n", chain[i].rect.x, chain[i].rect.y, chain[i].rect.width, chain[i].rect.height);
			ccv_array_push(regions, chain + i);
		} else if (chain[i].neighbors > 0) {
			//ccv_array_push(regions, chain + i);
			ccfree(chain[i].letters);
        }
	ccfree(chain);
	return regions;
}

#define less_than(a, b, aux) ((a)->center.x < (b)->center.x)
CCV_IMPLEMENT_QSORT(_ccv_sort_letters, ccv_letter_t*, less_than)
#undef less_than

static int _ccv_is_same_textline(const void* a, const void* b, void* data)
{
	ccv_textline_t* t1 = (ccv_textline_t*)a;
	ccv_textline_t* t2 = (ccv_textline_t*)b;
	int width = ccv_min(t1->rect.x + t1->rect.width, t2->rect.x + t2->rect.width) - ccv_max(t1->rect.x, t2->rect.x);
	int height = ccv_min(t1->rect.y + t1->rect.height, t2->rect.y + t2->rect.height) - ccv_max(t1->rect.y, t2->rect.y);
	/* overlapped 10% */
	double* thresh = (double*)data;
	return (width > 0 && height > 0 &&
			width * height > thresh[0] * ccv_max(t1->rect.width * t1->rect.height, t2->rect.width * t2->rect.height) &&
			width * height > thresh[1] * ccv_min(t1->rect.width * t1->rect.height, t2->rect.width * t2->rect.height));
}



ccv_array_t* ccv_swt_detect_textlines2(ccv_dense_matrix_t* a, ccv_swt_param_t params)
{

	int hr = a->rows * 2 / (params.min_height + params.max_height);
	int wr = a->cols * 2 / (params.min_height + params.max_height);
	double scale = pow(2., 1. / (params.interval + 1.));
	int next = params.interval + 1;
	int scale_upto = params.scale_invariant ? (int)(log((double)ccv_min(hr, wr)) / log(scale)) : 1;
	int k;
	//ccv_array_t* all_words = params.scale_invariant ? ccv_array_new(sizeof(ccv_rect_t), 2, 0) : 0;
	ccv_dense_matrix_t* phx = a;
	ccv_dense_matrix_t* pyr = a;
	//double cscale = 1.0;
    //ccv_write(a, "input.png", 0, CCV_IO_PNG_FILE, 0);

    ccv_array_t* all_textlines = ccv_array_new(sizeof(ccv_textline2_t), 0, 0);

	for (k = 0; k < scale_upto; k++)
	{
		// create down-sampled image on-demand because swt itself is very memory intensive
		if (k % next)
		{
			pyr = 0;
			int j = k % next;
			ccv_resample(phx, &pyr, 0, (int)(phx->rows / pow(scale, j)), (int)(phx->cols / pow(scale, j)), CCV_INTER_AREA);
		} else if (k > 0) {
			ccv_dense_matrix_t* pha = phx;
			phx = 0;
			ccv_sample_down(pha, &phx, 0, 0, 0);
			if (pha != a)
				ccv_matrix_free(pha);
			pyr = phx;
		}

		ccv_dense_matrix_t* swt = 0;

		params.direction = CCV_DARK_TO_BRIGHT;
		ccv_swt(pyr, &swt, 0, params);
        //ccv_write(swt, "swt.png", 0, CCV_IO_PNG_FILE, 0);

	    unsigned int elapsed_time = get_current_time();
		/* perform connected component analysis */
		ccv_array_t* lettersB = _ccv_swt_connected_letters(pyr, swt, params);
		ccv_matrix_free(swt);
		ccv_array_t* textline = _ccv_swt_merge_textline(lettersB, params);
		swt = 0;

        /*
		params.direction = CCV_BRIGHT_TO_DARK;
		ccv_swt(pyr, &swt, 0, params);
		ccv_array_t* lettersF = _ccv_swt_connected_letters(pyr, swt, params);
		ccv_matrix_free(swt);
		if (pyr != phx)
			ccv_matrix_free(pyr);
		ccv_array_t* textline2 = _ccv_swt_merge_textline(lettersF, params);
		for (i = 0; i < textline2->rnum; i++)
			ccv_array_push(textline, ccv_array_get(textline2, i));
		ccv_array_free(textline2);
        */


		ccv_array_t* idx = 0;

		int ntl = ccv_array_group(textline, &idx, _ccv_is_same_textline, params.same_word_thresh);
	    elapsed_time = get_current_time() - elapsed_time;
        printf("extract %ums, ntl=%d, rnum=%d\n", elapsed_time, ntl, textline->rnum);
        
        int j,k;
        //for (j=0; j<ntl; j++) 
        for (j=0; j<textline->rnum; j++) 
        {
            ccv_textline_t *line = (ccv_textline_t*)ccv_array_get(textline, j);
            if (line->neighbors < params.min_neighbors)
                continue;

            ccv_textline2_t line2;
            line2.neighbors = line->neighbors;
            line2.rect = line->rect;
            line2.letters = ccv_array_new(sizeof(ccv_letter_t), line->neighbors, 0);
            for (k=0; k<line->neighbors; k++) 
            {
                ccv_array_push(line2.letters, line->letters[k]);
            }
            ccv_array_push(all_textlines, &line2);
        }

		ccv_array_free(lettersB);
		//ccv_array_free(lettersF);
        ccv_array_free(textline);
    }


	if (phx != a)
		ccv_matrix_free(phx);

    return all_textlines;
}

void ccv_swt_free_textlines2(ccv_array_t *textlines)
{
    int j;
    for (j=0; j<textlines->rnum; j++) 
    {
        ccv_textline2_t *line = (ccv_textline2_t*)ccv_array_get(textlines, j);
        ccv_array_free(line->letters);
    }
    
    ccv_array_free(textlines);
}
