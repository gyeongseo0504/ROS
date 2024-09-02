#include <cstdio>
#include <vector>

using namespace std;

// 행렬 출력 함수 
void printMatrix(const vector<vector<int>>& matrix) //2차원 정수형 행렬 참
{
    for (const auto& row : matrix) //row,elem int형 임시변수 
    {
        for (int elem : row) 
        {
            printf("%d ", elem);
        }
        printf("\n");
    }
}

// 행렬 덧셈 함수
vector<vector<int>> addMatrix(const vector<vector<int>>& A, const vector<vector<int>>& B) 
{
    int m = A.size();    // 행렬 A의 행 수
    int n = A[0].size(); // 행렬 A의 열 수

    vector<vector<int>> C(m, vector<int>(n, 0));//같은 크기 C를 반환

    for (int i = 0; i < m; i++) //행의 계수를 계산하여 몇개의 요소인지 반환
    {
        for (int j = 0; j < n; j++) //열의 수를 계산하여 요소를 반환
        {
            C[i][j] = A[i][j] + B[i][j];//행렬 C에 m개의 행과 n개의 열을 가지며 두행렬을 더한다
        }
    }
    return C;
}

// 행렬 뺄셈 함수
vector<vector<int>> subtractMatrix(const vector<vector<int>>& A, const vector<vector<int>>& B) 
{
    int m = A.size();    // 행렬 A의 행 수
    int n = A[0].size(); // 행렬 A의 열 수

    vector<vector<int>> C(m, vector<int>(n, 0));

    for (int i = 0; i < m; i++) 
    {
        for (int j = 0; j < n; j++) 
        {
            C[i][j] = A[i][j] - B[i][j];//C행렬에 A,B 행렬 뺀값을 저장
        }
    }
    return C;
}

// 행렬 곱셈 함수
vector<vector<int>> multiplyMatrix(const vector<vector<int>>& A, const vector<vector<int>>& B) 
{
    int m = A.size();    // 행렬 A의 행 수
    int n = A[0].size(); // 행렬 A의 열 수 (B의 행 수)
    int p = B[0].size(); // 행렬 B의 열 수

    vector<vector<int>> C(m, vector<int>(p, 0));

    for (int i = 0; i < m; i++) 
    {
        for (int j = 0; j < p; j++) 
        {
            for (int k = 0; k < n; k++) 
            {
                C[i][j] += A[i][k] * B[k][j];//C[i][j]는 A의 i행과 B의 j열의 내적
            }
        }
    }
    return C;
}

// 행렬 전치 함수
vector<vector<int>> transposeMatrix(const vector<vector<int>>& A) 
{
    int m = A.size();    // 행렬 A의 행 수
    int n = A[0].size(); // 행렬 A의 열 수

    vector<vector<int>> T(n, vector<int>(m, 0)); //행렬 T는 원래 행렬 A의 열 수 n만큼의 행과, 행 수 m만큼의 열을 가짐

    for (int i = 0; i < m; i++) 
    {
        for (int j = 0; j < n; j++) 
        {
            T[j][i] = A[i][j];
        }
    }
    return T;
}

int main() 
{
    // 행렬 A 정의
    vector<vector<int>> A = {
        {1, 2, 3},
        {4, 5, 6}
    };

    // 행렬 B 정의 (덧셈과 뺄셈을 위한 같은 크기)
    vector<vector<int>> B = {
        {7, 8, 9},
        {10, 11, 12}
    };

    // 행렬 C 정의 (곱셈을 위한)
    vector<vector<int>> C = {
        {1, 4},
        {2, 5},
        {3, 6}
    };

    printf("행렬 A:\n");
    printMatrix(A);

    printf("\n행렬 B:\n");
    printMatrix(B);

    // 행렬 덧셈 
    vector<vector<int>> sumMatrix = addMatrix(A, B);
    printf("\n행렬 덧셈 (A + B):\n");
    printMatrix(sumMatrix);

    // 행렬 뺄셈 
    vector<vector<int>> diffMatrix = subtractMatrix(A, B);
    printf("\n행렬 뺄셈 (A - B):\n");
    printMatrix(diffMatrix);

    // 행렬 곱셈 
    vector<vector<int>> mulMatrix = multiplyMatrix(A, C);
    printf("\n행렬 곱셈 (A * C):\n");
    printMatrix(mulMatrix);

    // 행렬 전치 
    vector<vector<int>> transMatrix = transposeMatrix(A);
    printf("\n행렬 전치 (A^T):\n");
    printMatrix(transMatrix);

    return 0;
}
