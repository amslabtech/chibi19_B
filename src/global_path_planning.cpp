#include<stdio.h>
#include <stdlib.h>

const int row = 5;
const int column = 6;
const int direction = 8;

int delta[direction][2] = {{-1,0},
				   {0,-1},
				   {1,0 },
				   {0,1 }};
				   
char delta_name[4][3] = {"^","<","v",">"};

int positive_count(const int *a,const int size){
	int count = 0;
	for(int i=0;i<size;i++){
		if(a[i] > -1) count++;
	}
	return count;
}

int max_array(const int *a, int size,int &max){ //<<-return is i
	max=a[0];
	int max_i=0;
	for(int i=1;i<size;i++){
		if(max<a[i]){
			max = a[i];
			max_i = i;
		}
	}
	return max_i;
}
int min_array(const int *a, int size,int &min){ //<<positive value
	int min_i=0;
	for(int i=0;i<size;i++){
		if(a[i]>-1){
			min = a[i];
			min_i=i;
			break;
		}
		if(i==size-1) printf("error\n");
	}
	
	for(int i=0;i<size;i++){
		if(min>a[i] && a[i]>-1){
			min = a[i];
			min_i = i;
		}
	}
	return min_i;
}


void set_all(int array[row][column],const int setnum){
	for(int i=0;i<row;i++){
		for(int j=0;j<column;j++){
			array[i][j] = setnum;
		}
	}
}

void set_heuristic(int array[row][column],const int goal[2]){
	array[goal[0]][goal[1]] = 0;
	for(int i=0;i<row;i++){
		for(int j=0;j<column;j++){
			array[i][j] = abs(goal[0]-i)+abs(goal[1]-j);
		}
	}
}

void show_array(int array[row][column]){
	printf("[[");
	for(int i=0;i<row;i++){
		for(int j=0;j<column;j++){
			printf("%2d",array[i][j]);
			if(j != column-1) printf(",");
		}
		printf("]");
		if(i != row-1) printf("\n [");
	}
	printf("]\n");
}



int search(const int grid[row][column],const int heuristic_grid[row][column],
	int path[row][column], const int init[2],const int goal[2])
{
	int move_history[row*column][2];
	int move_count = 0;
	int a[4]= {-1};
	int y = init[0];
	int x = init[1];
	int d;
	int move_direction;
	int f;
	
	path[y][x] = 0;
	for(f=1;f<row*column;f++){
		if(x == goal[1] && y == goal[0]){
			printf("success!!\n");
			break;
		}
		for(int i=0;i<4;i++){
			if(-1<(x+delta[i][1]) && column>(x+delta[i][1]) && -1<(y+delta[i][0]) && row>(y+delta[i][0])){
				if(grid[y+delta[i][0]][x+delta[i][1]] != 1){
					if(path[y+delta[i][0]][x+delta[i][1]] == -1)
						a[i] = heuristic_grid[y+delta[i][0]][x+delta[i][1]];
					else a[i] = -1;
				}
				else a[i] = -1;
			}
			else a[i] = -1;
		}
		
		if(positive_count(a,4)>1){
			move_history[move_count][0]=y;
			move_history[move_count][1]=x;
			move_count++;
		}
		
		
		if(!positive_count(a,4)){
			if(!move_count){
				printf("----analysis is impossible----\n");
				break;
			}
			move_count--;
			y = move_history[move_count][0];
			x = move_history[move_count][1];
			f--;
		}
		
		else{
			move_direction = min_array(a,4,d);
			x = x+delta[move_direction][1];
			y = y+delta[move_direction][0];
			path[y][x] = f;
		}
	}
	f--;
	return f;
}

int main(void){
	
	int grid[row][column] = {{0, 1, 0, 0, 0, 0},
							 {0, 1, 0, 1, 1, 0},
							 {0, 0, 0, 1, 1, 0},
							 {1, 1, 0, 1, 1, 0},
							 {0, 0, 0, 0, 1, 0}};
	

	int heuristic_grid[row][column];
	int path[row][column];
	set_all(path,-1);
	
	const int init[2] = {0,0};
	const int goal[2] = {4,5};
	//int cost = 1;
	
	set_heuristic(heuristic_grid,goal);
	
	int f = search(grid, heuristic_grid,path,init,goal);
	printf("grid = \n"); show_array(grid);
	printf("heuristic_grid = \n"); show_array(heuristic_grid);
	printf("path = \n"); show_array(path);
	printf("f = %d\n",f);
	
}
