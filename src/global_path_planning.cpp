#include "ros/ros.h"

const int row = 5;
const int column = 6;
const int direction = 8;

int delta[direction][2] = {{-1,0,},
					{-1,-1},
					{0,-1},
					{1,-1},
				   	{1,0 },
					{1,1 },
					{0,1 },
					{-1,1}};
				   
float delta_cost[direction] = {1.0,sqrtf(2.0),1.0,sqrtf(2.0),1.0,sqrtf(2.0),1.0,sqrtf(2.0)}; 

char delta_name[direction][5] = {"^","「","<","L","v","」",">","7"};

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
		if(i==size-1) min = -1;//printf("error\n");
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

void fset_all(float array[row][column],const float setnum){
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

void show_farray(float array[row][column]){
     printf("[[");
     for(int i=0;i<row;i++){
         for(int j=0;j<column;j++){
             printf("%4.1f",array[i][j]);
             if(j != column-1) printf(",");
         }
         printf("]");
         if(i != row-1) printf("\n [");
     }   
     printf("]\n");
}

void show_cost_grid(int array[row][column]){
	for(int i=0;i<row;i++){
		for(int j=0;j<column;j++){
			if(array[i][j] == -1) printf("■ ");
			else printf("%2s",delta_name[array[i][j]]);
		}
		if(i != row-1) printf("\n");
	}
	printf("\n");
}

int search(const int grid[row][column],const int heuristic_grid[row][column],
	float cost_grid[row][column], int temporary_path[][3], const int init[2],const int goal[2])
{
	int move_history[row*column][2];
	int selected_move = 0;
	int a[direction]= {-1};
	int eight_cost[direction];
	int y = init[0];
	int x = init[1];
	int d,min_c,step,home;
	int move_direction;

	cost_grid[y][x] = 0;
	for(int f=0;f<row*column;f++){
		if(x == goal[1] && y == goal[0]){
			cost_grid[y][x] = step+1;
			temporary_path[step+1][0] = y;
			temporary_path[step+1][1] = x;
			temporary_path[step+1][2] = move_direction;
			//printf("success!!\n");
			return cost_grid[y][x];
		}
		for(int i=0;i<direction;i++){
			if(-1<(x+delta[i][1]) && column>(x+delta[i][1]) && -1<(y+delta[i][0]) && row>(y+delta[i][0])){
				eight_cost[i] = cost_grid[y+delta[i][0]][x+delta[i][1]];
				if(grid[y+delta[i][0]][x+delta[i][1]] != 1){
					if(cost_grid[y+delta[i][0]][x+delta[i][1]] < 0)
						a[i] = heuristic_grid[y+delta[i][0]][x+delta[i][1]];
					else a[i] = -1;
				}
				else a[i] = -1;
			}
			else {
				a[i] = -1;
				eight_cost[i]=-1;
			}
		}
		
		if(positive_count(a,direction)>1){
			move_history[selected_move][0]=y;
			move_history[selected_move][1]=x;
			selected_move++;
		}
		
		
		if(!positive_count(a,direction)){
			if(!selected_move){
				printf("----analysis is impossible----\n");
				return 0;
			}
			selected_move--;
			y = move_history[selected_move][0];
			x = move_history[selected_move][1];
		}
		
		else{
			home = min_array(eight_cost,direction,min_c);//from home
			step = min_c+1;
			if(step>0) temporary_path[step-1][2] = (home+4)%9;
			cost_grid[y][x] = step;
			move_direction = min_array(a,direction,d);
			temporary_path[step][0] = y;
			temporary_path[step][1] = x;
			temporary_path[step][2] = move_direction;
			x = x+delta[move_direction][1];
			y = y+delta[move_direction][0];
		}
	}
}

int set_path(int temporary_path[][3], int result_path[][3],int step)
{
	for(int i=0;i<step;i++){
		result_path[i][0] = temporary_path[i][0];
		result_path[i][1] = temporary_path[i][1];
		result_path[i][2] = temporary_path[i][2];
		printf("path = [%3d,%3d,%3d]\n",result_path[i][0],result_path[i][1],result_path[i][2]);
	}

}
	

int main(void){
	
	int grid[row][column] = {{0, 1, 0, 0, 0, 0},
							 {0, 0, 0, 1, 1, 0},
							 {0, 1, 0, 1, 1, 0},
							 {1, 1, 0, 1, 1, 0},
							 {0, 0, 0, 0, 1, 0}};
	

	int heuristic_grid[row][column];
	float cost_grid[row][column];
	int temporary_path[row*column][3];
	fset_all(cost_grid,-1);
	
	const int init[2] = {0,0};
	const int goal[2] = {4,5};
	
	set_heuristic(heuristic_grid,goal);
	
	const int step = search(grid, heuristic_grid,cost_grid,temporary_path,init,goal) + 1;
	int result_path[step][3];

	set_path(temporary_path, result_path, step);
	printf("grid = \n"); show_array(grid);
	//printf("heuristic_grid = \n"); show_array(heuristic_grid);
	printf("cost_grid = \n"); show_farray(cost_grid);
}
