#include <iostream>
#include <vector>

using namespace std;
using matrix = vector<vector<double>>;

matrix multiply(matrix A, matrix B)
{
    matrix C(A.size(), vector<double>(B[0].size(), 0));
    for (int i = 0; i < A.size(); i++)
    {
        for (int j = 0; j < B[0].size(); j++)
        {
            for (int k = 0; k < A[0].size(); k++)
            {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return C;
}

void print(matrix A)
{
    for (int i = 0; i < A.size(); i++)
    {
        for (int j = 0; j < A[0].size(); j++)
        {
            cout << A[i][j] << " ";
        }
        cout << endl;
    }
}

int main()
{
    matrix matA = {
        {1, 2, 3},
        {4, 5, 6},
        {7, 8, 9}
    };
    matrix matB = {
        {1, 2, 3},
        {1, 2, 3},
        {1, 2, 3}
    };

    matrix matC = multiply(matA, matB);
    print(matC);
    /* Result:
        6 12 18
        15 30 45
        24 48 72
    */
    return 0;
}