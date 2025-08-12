#include <stdio.h>


#define FORGROUND	255	// Blob½Ã ¹°Ã¼ »ö±ò
#define BACKGROUND	0	// Blob½Ã ¹è°æ »ö±ò
#define MAX_NO_BLOB 1000 // ÃÖ´ë Blob ¼ö 


typedef unsigned char BYTE;
typedef unsigned int UINT; 

typedef struct _CSize
{
	int cx;
	int cy;	
} CSize;

typedef struct _CRect
{
	int top;
	int bottom;	
    int right;
    int left;
} CRect;


void grass(int *projection_temp, BYTE *Label, int size_projection, int i, int no_label);
int Blob_Analysis2(BYTE *Binary_Image, UINT *Label_Image, CSize image_size,unsigned int min_area,unsigned long max_area);
int push(int *stackx, int *stacky, int vx, int vy, int *top,int maxsize);
int pop(int *stackx, int *stacky, int *vx, int *vy, int *top);
void Projection_V(BYTE *In_Image, CSize image_size, CRect ROI, double *projection_v);
void Find_ProjectData_Center_Position(double *projection, int size_projection, int threshold, double *position, int *no_pad, CSize* Projection_Blob);


void grass(int *projection_temp, BYTE *Label, int size_projection, int i, int no_label)
{

	int k, index;
	int height;
	int width;

	height = 1;
	width = size_projection;


	for (k = i - 1; k <= i + 1; k++)
	{
		// ¿µ»óÀÇ °æ°è¸¦ ¹þ¾î³ª¸é ¶óº§¸µÇÏÁö ¾ÊÀ½ 
		if (k<0 || k > width) continue;

		index = k;
		// ¾ÆÁ÷ ¹æ¹®ÇÏÁö ¾ÊÀº ÇÈ¼¿ÀÌ°í °ªÀÌ 255¶ó¸é ¶óº§¸µÇÔ 
		if (*(projection_temp + k) == 255 && *(Label + index) == 0)
		{
			*(Label + index) = no_label;

			grass(projection_temp, Label, size_projection, k, no_label);
		}

	}




}


int push(int *stackx, int *stacky, int vx, int vy, int *top,int maxsize)
{
	if(*top>=maxsize) return(-1);
	(*top)++;
	stackx[*top]=vx;
	stacky[*top]=vy;
	return(1);
}

int pop(int *stackx, int *stacky, int *vx, int *vy, int *top)
{
	if(*top<=0) return(-1);
	*vx=stackx[*top];
	*vy=stacky[*top];
	(*top)--;
	return(1);
}

int Blob_Analysis2(BYTE *Binary_Image, UINT *Label_Image, CSize image_size,unsigned int min_area,unsigned long max_area)
{

	UINT *label_image_temp;
	
	int i,j,m,n,top; 
	UINT  curColor=0;
	int r,c;
	int k;
	unsigned long area,BlobArea[MAX_NO_BLOB];
	
	int no_blob=0;
	int width = image_size.cx;
	int height =image_size.cy;
	
	int* stackx=new int [height*width];
	int* stacky=new int [height*width];
	
	label_image_temp = new UINT[height*width];
	

	for(i=0; i<height*width; i++)  label_image_temp[i] = Label_Image[i]=0; //ë©ëª¨ë¦?ì´ê¸°??
	
	//    max_area = height*width; // 	min_area = 1000;
	
	
	//	for(i=0; i<5; i++) 	
	//	Erode(binary_image,binary_image,image_size);
	
	
	for(i=0,no_blob=0; i<height; i++) 
	{
		for(j=0; j<width; j++) 
		{	
			// ?´ë? ë°©ë¬¸???ì´ê±°ë ?½ì?ê°ì´ 255ê° ?ë?¼ë©´ ì²ë¦¬ ?í¨ 
			if(label_image_temp[i*width+j]!=0 || Binary_Image[i*width+j]!=FORGROUND) continue;
			
			r=i; 
			c=j;
			top=0; area=1;
			curColor++;
			
			while(1) 
			{
GRASSFIRE:
			for(m=r-1; m<=r+1; m++)
			{
				for(n=c-1; n<=c+1; n++) 
				{
				
					if(m<0 || m>=height || n<0 || n>=width) continue;
					
					if(Binary_Image[m*width+n]==FORGROUND && label_image_temp[m*width+n]==0)
					{
						label_image_temp[m*width+n]=curColor;  // ?ì¬ ?¼ë²¨ë¡?ë§í¬ 
						if(push(stackx,stacky,m,n,&top, width*height)==-1) continue;
						r=m; 
						c=n;
						area++;
						goto GRASSFIRE;
					}
				}
			}
			if(pop(stackx,stacky,&r,&c,&top)==-1) break;
			}
			
			if(curColor<MAX_NO_BLOB) BlobArea[curColor] = area;
			
		//	printf("Blob Area[%d] = %d\n", curColor,area);
			
			
			
			
		
			if( (area >= min_area) &&(area <= max_area))
			{
				
				for(k=0; k<height*width; k++)  
				{
					if(label_image_temp[k]== curColor) Label_Image[k] = no_blob+1;
				}
				
				//	printf("Blob[%d] Area %d\n", no_blob, area);
				no_blob++;
			}
			
			
			// --------------------------------------------------------
			
		}
	}
	
	
	delete label_image_temp;
	delete []stackx; delete []stacky;
	
	return no_blob;

}

void Projection_V(BYTE *In_Image, CSize image_size, CRect ROI, double *projection_v)
{

	int x, y;
	int height = image_size.cy;
	int width = image_size.cx;

	memset(projection_v, 0, width*sizeof(double));

	for (x = ROI.left; x < ROI.right; x++)
	{
		for (y = ROI.top; y < ROI.bottom; y++)
		{
			*(projection_v + x) += (double)(*(In_Image + y*width + x)) / (255.0);
		}
	
	}

}


void Find_ProjectData_Center_Position(double *projection, int size_projection, int threshold, double *position, int *no_pad, CSize* Projection_Blob)
{

	int x, k;
	int start_x, no_data;
	double position_sum;
	int *projection_temp;
	BYTE *Label;
	int no_label = 0;
	int max_pos, min_pos;
	int index_cnt = 0;

	projection_temp = new int[size_projection];
	Label = new BYTE[size_projection];

	/*
	for (x = 0; x < size_projection; x++)
	{
	*(Label + x) = 0;
	*(projection_temp + x) = *(projection + x);
	}
	*/
	memset(Label, 0, size_projection*sizeof(BYTE));

	memset(projection_temp, 0, size_projection*sizeof(int));


	for (x = 0; x < size_projection; x++)
	{

		if (*(projection + x) >= threshold)
		{
			*(projection_temp + x) = 255;
		}
		else
		{
			*(projection_temp + x) = 0;
		}
	}


	no_label = 0;

	for (x = 0; x < size_projection; x++)
	{
		if (*(projection_temp + x) == 255 && *(Label + x) == 0)
		{
			no_label++;
			*(Label + x) = no_label;
			grass(projection_temp, Label, size_projection, x, no_label);
		}
	}



	start_x = 0;

	no_label = (no_label > 48) ? 48 : no_label;

	for (k = 1, index_cnt = 0; k <= no_label; k++)
	{
		position_sum = 0;
		no_data = 0;
		max_pos = 0, min_pos = size_projection;
		for (x = start_x; x < size_projection; x++)
		{
			if (*(Label + x) == k)
			{
				position_sum += (double)x;
				no_data++;
				start_x = x;

				if (max_pos <= x)  max_pos = x;
				if (min_pos >= x)  min_pos = x;
			}

		}

		*(position + k - 1) = position_sum / no_data;
		Projection_Blob[index_cnt].cx = min_pos;
		Projection_Blob[index_cnt].cy = max_pos;

		if ((max_pos - min_pos) > 15)
		{
			index_cnt++;

		}

	}
	*no_pad = index_cnt;

	delete[]projection_temp;
	delete[]Label;
}





