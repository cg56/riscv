#include <stdio.h>

int main(int argc, const char *argv[])
{
    FILE *fp = fopen(argv[1], "r");
    if (fp == 0)
        printf("Error");

    unsigned int word;
    int addr = 0;

    while (fread(&word, 1, 4, fp) == 4)
        printf("%4d:\tword <= 32'h%08x;\n", addr++, word);

    fclose(fp);
}

