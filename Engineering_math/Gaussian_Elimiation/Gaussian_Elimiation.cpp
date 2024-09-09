#include <stdio.h>
#include <math.h>
#include <iostream>
#include <vector>
using namespace std;

double cal_pivot(const vector<vector<double>>& C, int row1, int row2, int column) // row1 is base
{
    double pivot;

    pivot = C[row1][column] / C[row2][column];

    return pivot;
}

void matrix_row_operation(vector<vector<double>>& C, int row1, int row2, double pivot)
{
    for (int k = 0; k < C[0].size(); k++)
    {
        C[row2][k] = pivot * C[row2][k];
        printf("%6.3lf ", C[row2][k]);
    }
    printf("\n");

    for (int k = 0; k < C[0].size(); k++)
    {
        C[row2][k] = C[row1][k] - C[row2][k];
        printf("%6.3lf ", C[row2][k]);
    }
    printf("\n");
}

void print_Matrix(const vector<vector<double>>& C)
{
    printf("\n");
    for (int i = 0; i < C.size(); i++)
    {
        for (int j = 0; j < C[0].size(); j++)
        {
            printf("%6.3lf ", C[i][j]);
        }
        printf("\n");
    }
    printf("\n");
}

int main(void)
{
    double pivot = 0.0;

    vector<vector<double>> A =
    {
        {2, 1, -1},
        {-3, -1, 2},
        {-2, 1, 2}
    };
    vector<vector<double>> B =
    {
        {8},
        {-11},
        {-3}
    };

    vector<vector<double>> C = A;

    int mA = A.size();
    int nA = A[0].size();

    for(int i = 0; i < C.size(); i++)
    {
        C[i].push_back(B[i][0]);
    }
    print_Matrix(C);

    pivot = cal_pivot(C,0,1,0);
    printf("0 1 0 %6.3lf \n", pivot);

    pivot = cal_pivot(C,1,2,1);
    printf("1 2 1 %6.3lf \n", pivot);
    printf("\n");

    for(int i = 1; i < C.size(); i++)
    {
        pivot = cal_pivot(C,0,i,0);
        matrix_row_operation(C,0,i,pivot);
        print_Matrix(C);
    }

    printf("--------------------------------------------------\n");
    printf("-----------------------예?------------------------\n");
    printf("--------------------------------------------------\n");

    // Second iteration
    for(int i = 2; i < C.size(); i++)
    {
        pivot = cal_pivot(C,1,i,1);
        matrix_row_operation(C,1,i,pivot);
        print_Matrix(C);
    }

    return 0;
}
