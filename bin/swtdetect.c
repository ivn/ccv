#include "ccv.h"
#include <sys/time.h>
#include <ctype.h>




int main(int argc, char** argv)
{
	//ccv_enable_cache(1024*1024*1024);
	ccv_enable_default_cache();
	ccv_dense_matrix_t* image = 0;
	ccv_dense_matrix_t* image2 = 0;
	ccv_dense_matrix_t* image3 = 0;
	ccv_read(argv[1], &image, CCV_IO_GRAY | CCV_IO_ANY_FILE);
	if (image != 0)
	{
        ipp_resize(image, &image2);
        ipp_gauss_filter(image2, &image3);
        ccv_write(image3, "resized.png", 0, CCV_IO_PNG_FILE, 0);
		unsigned int elapsed_time = get_current_time();
		ccv_array_t* textlines = ccv_swt_detect_textlines2(image2, ccv_swt_default_params);
		elapsed_time = get_current_time() - elapsed_time;
		if (textlines)
		{
			int i;
			for (i = 0; i < textlines->rnum; i++)
			{
				//ccv_rect_t* rect = (ccv_rect_t*)ccv_array_get(textlines, i);
				//printf("%d %d %d %d\n", rect->x, rect->y, rect->width, rect->height);

                ccv_textline_t *line = (ccv_textline_t*)ccv_array_get(textlines, i);
                printf("%d %d %d %d\n", 
                    line->rect.x,
                    line->rect.y,
                    line->rect.width,
                    line->rect.height
                    );

                for (int j=0; j<line->neighbors; j++) {
                    ccv_letter_t *letter = (ccv_letter_t*) ccv_array_get((ccv_array_t*)line->letters, j);

                    printf("\t%d %d %d %d\n", 
                        letter->rect.x,
                        letter->rect.y,
                        letter->rect.width,
                        letter->rect.height
                        );

                    fflush(stdout);
                }
                
			}
			printf("total : %d in time %dms\n", textlines->rnum, elapsed_time);
			ccv_swt_free_textlines2(textlines);
		}
		ccv_matrix_free(image);
		ccv_matrix_free(image2);
		ccv_matrix_free(image3);
	} /*else {
		FILE* r = fopen(argv[1], "rt");
		if (argc == 3)
			chdir(argv[2]);
		if(r)
		{
			size_t len = 1024;
			char* file = (char*)malloc(len);
			ssize_t read;
			while((read = getline(&file, &len, r)) != -1)
			{
				while(read > 1 && isspace(file[read - 1]))
					read--;
				file[read] = 0;
				image = 0;
				printf("%s\n", file);
				ccv_read(file, &image, CCV_IO_GRAY | CCV_IO_ANY_FILE);
				ccv_array_t* textlines = ccv_swt_detect_textlines(image, ccv_swt_default_params);
				int i;
				for (i = 0; i < textlines->rnum; i++)
				{
					ccv_rect_t* rect = (ccv_rect_t*)ccv_array_get(textlines, i);
					printf("%d %d %d %d\n", rect->x, rect->y, rect->width, rect->height);
				}
				ccv_array_free(textlines);
				ccv_matrix_free(image);
			}
			free(file);
			fclose(r);
		}
	}
    */
	ccv_drain_cache();
	return 0;
}

