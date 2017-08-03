#include <stdio.h>
#include "matrix.h"

void mat_print(int rows, int cols, const double *data, const char *desc) {
    printf("%s:\n", desc);

    for (int i = 0; i < rows; ++i) {
        printf("    [ ");

        for (int j = 0; j < cols; ++j)
            printf("%+9.4f ", *data++);

        printf("]\n");
    }

    printf("\n");
}

void mat_mult(int lrows, int lcols, int rcols, const double *left,
                                               const double *right,
                                               double *result) {
    for (int i = 0; i < lrows; ++i)
        for (int j = 0; j < rcols; ++j) {
            *result = 0;

            for (int k = 0; k < lcols; ++k)
                *result += left[k + i * lcols] * right[j + k * rcols];

            ++result;
        }
}
