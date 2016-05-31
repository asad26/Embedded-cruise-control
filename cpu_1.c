#include <stdio.h>
#include <stdint.h>
#include "system.h"
#include "fifo.h"

#define DEBUG 0

enum {
        GRAYSCALE_RESIZE,
        SOBEL_ASCII,
};

struct job {
        uint8_t task;
        uint8_t xdim;
        uint8_t ydim;
        unsigned segment : 4;
        unsigned image_num : 2;
		unsigned align : 2;
};

/*const uint8_t *memlocs[3] = 
{
        SHARED_ONCHIP_BASE,
        SHARED_ONCHIP_BASE + 4000,
        SHARED_ONCHIP_BASE + 6000
};*/

void image_detect_edges(uint8_t *src, uint8_t *dest, uint8_t segment, uint8_t nr_segments);
void image_to_grayscale_resize(uint8_t *src, uint8_t *dest, uint8_t segment, uint8_t nr_segments);

int main()
{
        uint16_t src, dst;
        struct job job;
		uint8_t xdim, ydim;
                
        while (1) {
                /* Get job from fifo */
                while (fifo_read(FIFO0, &job) == -1)
                        ;

                if (job.task == GRAYSCALE_RESIZE) {
						xdim = job.xdim;
						ydim = job.ydim;
						#if DEBUG == 1
						printf("Dimensions [%d x %d] - segment %d\n", xdim, ydim, job.segment+1);
						#endif
						
						if(ydim==32 && xdim==32) {
								src=0;
								dst=6150;
								if(job.image_num==1)
										dst=6406;
								ydim=ydim/4;
								src += ((job.segment)*768);
								dst += ((job.segment)*64);
						}
						else {
								src=0;
								dst=6150;
								ydim=ydim/4;
								uint8_t ydim2=ydim;
								if((ydim & 1) != 0) {
										ydim2=ydim+1;
										if(job.segment==0 || job.segment==1 || job.segment==2) {
												ydim=ydim+1;
										}
										else {
											ydim=ydim-1;
										}
								}
								src+=xdim*ydim2*3*job.segment;
								dst+=xdim*ydim2/4*job.segment;
						}
                        image_to_grayscale_resize((uint8_t *)(SHARED_ONCHIP_BASE + src + job.align), (uint8_t *)(SHARED_ONCHIP_BASE + (dst)), xdim, ydim);
						#if DEBUG == 1
						printf("Part [%d x %d]\n", xdim, ydim);
						printf("Source %d\n", src);
						printf("Destination %d\n", dst);
                        printf("Job done(1)!\n");
						#endif
                        /* Write back GRAYSCALE_RESIZE to let cpu0 know we finished the job */
                        fifo_write(FIFO1, GRAYSCALE_RESIZE);
                }
                
                if (job.task == SOBEL_ASCII) {
						xdim = job.xdim;
						ydim = job.ydim;
						#if DEBUG == 1
						printf("Dimensions [%d x %d] - segment %d\n", xdim, ydim, job.segment+1);
						#endif
						
						src=6150;
						dst=6662;
						if(job.image_num==1)
										dst=6858;
						
						if(ydim==16 && xdim==16) {
								if(job.segment == 0) {
										ydim = 4;
										src+=16;
								}
								if(job.segment == 1) {
										ydim = 4;
										src+=64+16;
										dst+=56;
								}
								if(job.segment == 2) {
										ydim = 3;
										src+=128+16;
										dst+=112;
								}
								if(job.segment == 3) {
										ydim = 3;
										src+=176+16;
										dst+=154;
								}
						}
						else {
							uint8_t ydim2=ydim;
							ydim = ydim/4;
							src+=xdim*ydim*job.segment;
							dst+=(xdim-2)*ydim*job.segment;
							if(job.segment == 3) {
									
									ydim = ydim2-(((int)ydim2/4)*3);
									
							}
						}
						#if DEBUG == 1
						printf("Part [%d x %d]\n", xdim, ydim);
						printf("Source %d\n", src);
						printf("Destination %d\n", dst);
						#endif
                        image_detect_edges((uint8_t *)(SHARED_ONCHIP_BASE + src + job.align), (uint8_t *)(SHARED_ONCHIP_BASE + dst), xdim, ydim);
						#if DEBUG == 1
                        printf("Job done(2)!\n");
						#endif
                        /* Write back SOBEL_ASCII to let cpu0 know we finished the job */
                        fifo_write(FIFO1, SOBEL_ASCII);
                }
        }
                

        return 0;
}

inline int abs(int x)
{
        return (x > 0) ? x : -x;
}

void image_detect_edges(uint8_t *src, uint8_t *dst, uint8_t xdim, uint8_t ydim)
{
        uint8_t x;
        uint16_t y;
        int32_t pixel_x, pixel_y;
		uint16_t i=0;
		uint16_t offset=1;
		uint16_t root, limit;
		
		const int ascii[] = {64, 37, 38, 36, 35,111, 115, 197, 63, 33, 42, 43, 61, 95, 45, 32};
        
        /* Ignore borders - start at 1 and stop at end-1 */
        for (y = 0; y < ydim; y++) {
				limit=offset+xdim - 2;
                for (; offset < limit; offset++) {
						pixel_x =
                                src[offset - xdim - 1]
                                - src[offset - xdim + 1]
                                + src[offset - 1] 
                                + src[offset - 1]
                                - src[offset + 1] 
                                - src[offset + 1]
                                + src[offset + xdim - 1]
                                - src[offset + xdim + 1];
                        pixel_y =
                                src[offset - xdim - 1]
                                + src[offset - xdim]
                                + src[offset - xdim]
                                + src[offset - xdim + 1]
                                - src[offset + xdim - 1]
                                - src[offset + xdim]
                                - src[offset + xdim]
                                - src[offset + xdim + 1];

                        root = abs(pixel_x) + abs(pixel_y);
						dst[i++] = (root > 255) ? 32 : ascii[root >> 4];
                }
				offset+=2;
        }
}

void image_to_grayscale_resize(uint8_t *src, uint8_t *dst, uint8_t xdim, uint8_t ydim) {
			
	    int j, p=0;
		int l1 = xdim*ydim*3;
		int l2;
		uint8_t xdim3=xdim*3;
		
        for(j = 0; j < l1; j += xdim3) {
				l2=j+xdim3;
                for(; j < l2; j += 6) {

				dst[p] = (
				(((((src[j] << 5)
				+ (src[j] << 3)) >> 3)
				+ (((src[j+1] << 5)
				+ (src[j+1] << 2)) >> 2)
				+ (src[j+2] << 1)))
				+ (((((src[j+3] << 5)
				+ (src[j+3] << 3)) >> 3)
				+ (((src[j+4] << 5)
				+ (src[j+4] << 2)) >> 2)
				+ (src[j+5] << 1)))
				+ (((((src[j+xdim3] << 5)
				+ (src[j+xdim3] << 3)) >> 3)
				+ (((src[j+xdim3+1] << 5)
				+ (src[j+xdim3+1] << 2)) >> 2)
				+ (src[j+xdim3+2] << 1)))
				+ (((((src[j+xdim3+3] << 5)
				+ (src[j+xdim3+3] << 3)) >> 3)
				+ (((src[j+xdim3+4] << 5)
				+ (src[j+xdim3+4] << 2)) >> 2)
				+ (src[j+xdim3+5] << 1)))
				) >> 6;
				
				p++;
				}
		}
}
