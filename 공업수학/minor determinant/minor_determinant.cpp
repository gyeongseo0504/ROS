#include <iostream>
#include <vector>
#include <iomanip>

using namespace std;

void printMatrix(const vector<vector<int>>& matrix) {
    if (matrix.empty()) {
        cout << "빈 행렬입니다." << endl;
        return;
    }
    
    int max_width = 0;
    for (const auto& row : matrix) {
        for (int elem : row) {
            int width = to_string(elem).size();
            if (width > max_width) {
                max_width = width;
            }
        }
    }
    max_width = max(max_width, 4);

    for (const auto& row : matrix) {
        for (int elem : row) {
            cout << setw(max_width) << elem << ' ';
        }
        cout << endl;
    }
}

vector<vector<int>> addMatrix(const vector<vector<int>>& A, const vector<vector<int>>& B) {
    int m = A.size();
    int n = A[0].size();
    
    vector<vector<int>> D(m, vector<int>(n, 0));
    
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            D[i][j] = A[i][j] + B[i][j];
        }
    }
    return D;
}

vector<vector<int>> subtractMatrix(const vector<vector<int>>& A, const vector<vector<int>>& B) {
    int m = A.size();
    int n = A[0].size();
    
    vector<vector<int>> D(m, vector<int>(n, 0));
    
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            D[i][j] = A[i][j] - B[i][j];
        }
    }
    return D;
}

vector<vector<int>> multiplyMatrix(const vector<vector<int>>& A, const vector<vector<int>>& B) {
    int m = A.size();   
    int n = A[0].size(); 
    int p = B[0].size(); 
    
    vector<vector<int>> D(m, vector<int>(p, 0));
    
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < p; j++) {
            for (int k = 0; k < n; k++) {
                D[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return D;
}

vector<vector<int>> transposeMatrix(const vector<vector<int>>& A) {
    int m = A.size();   
    int n = A[0].size();
    
    vector<vector<int>> B(n, vector<int>(m, 0));
    
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            B[j][i] = A[i][j];
        }
    }
    return B;
}

int determinant3x3(const vector<vector<int>>& A) {
    int det = 0;
    if (A.size() != 3 || A[0].size() != 3) {
        cout << "3x3 행렬이 아닙니다." << endl;
        return 0;
    }
    else
    {
        det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
              A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
              A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    }
    return det;
}

int main() {
    vector<vector<int>> A = {
        {1, 2, 3},
        {4, 5, 6}
    };

    vector<vector<int>> B = {
        {7, 8, 9},
        {10, 11, 12}
    };

    vector<vector<int>> A_transposed = transposeMatrix(A);
    vector<vector<int>> B_transposed = transposeMatrix(B);

    vector<vector<int>> C = {
        {5, 7, 3},
        {4, 1, 6},
        {7, 3, 9}
    };

    cout << "행렬 A:" << endl;
    printMatrix(A);

    cout << "\n행렬 B:" << endl;
    printMatrix(B);
    
    vector<vector<int>> D_add = addMatrix(A, B);
    cout << "\n행렬 A + B:" << endl;
    printMatrix(D_add);

    vector<vector<int>> D_sub = subtractMatrix(A, B);
    cout << "\n행렬 A - B:" << endl;
    printMatrix(D_sub);

    vector<vector<int>> D_mul = multiplyMatrix(A, B_transposed);
    cout << "\n행렬 A * B의 전치:" << endl;
    printMatrix(D_mul);

    cout << "\n행렬 A의 전치:" << endl;
    printMatrix(A_transposed);

    cout << "\n행렬 B의 전치:" << endl;
    printMatrix(B_transposed);

    cout << "\n행렬 C:" << endl;
    printMatrix(C);

    int det_C = determinant3x3(C);
    cout << "\n행렬 C의 DET: " << det_C << endl;

    return 0;
}