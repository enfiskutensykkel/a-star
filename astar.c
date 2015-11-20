#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <math.h>
#include <float.h>

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))


char* map = NULL;
uint32_t width = 0, height = 0;

uint8_t* map_exits = NULL;
uint8_t* closed_list = NULL;

double* distances = NULL;
uint32_t* path = NULL;
uint32_t* heap = NULL;
uint32_t heap_size = 0;

int is_marked(uint8_t* bitmap, uint32_t idx)
{
    return !!(bitmap[idx >> 3] & (1 << (idx & 7)));
}

void mark(uint8_t* bitmap, uint32_t idx)
{
    bitmap[idx >> 3] |= 1 << (idx & 7);
}

void heap_insert(uint32_t point)
{
    uint32_t hole = ++heap_size;

    for (; hole > 1 && distances[point] < distances[heap[hole >> 1]]; hole >>= 1)
    {
        heap[hole] = heap[hole >> 1];
    }

    heap[hole] = point;
}

uint32_t heap_remove()
{
    uint32_t remove = heap[1];

    uint32_t node = heap[1] = heap[heap_size--];

    uint32_t hole = 1, child;

    for (; (hole << 1) <= heap_size; hole = child)
    {
        child = hole << 1;

        if (child != heap_size && distances[heap[child + 1]] < distances[heap[child]])
        {
            ++child;
        }

        if (distances[heap[child]] < distances[node])
        {
            heap[hole] = heap[child];
        }
        else
        {
            break;
        }
    }

    heap[hole] = node;
    return remove;
}


double manhatten(int64_t sx, int64_t sy, int64_t dx, int64_t dy) 
{
    int64_t tx = llabs(sx - dx);
    int64_t ty = llabs(sy - dy);

    return 2.0 * (tx + ty);
}

double chebyshev(int64_t sx, int64_t sy, int64_t dx, int64_t dy) 
{
    double D = 14.0;

    int64_t tx = llabs(sx - dx);
    int64_t ty = llabs(sy - dy);
    return D * (tx + ty) + (D * 2.0 - 2.0 * D) * MIN(tx, ty);
} 

uint32_t find_closest_exit(uint32_t point, uint32_t e_x, uint32_t e_y)
{
    distances[point] = 0;
    path[point] = point;

    heap_insert(point);

    while (heap_size > 0)
    {
        uint32_t u = heap_remove();
        int64_t x = u % width;
        int64_t y = u / height;

        //if (is_marked(map_exits, u))
        if (x == e_x && y == e_y)
        {
            // found a way out
            return u;
        }

        // remove u from open list and insert into closed list
        mark(closed_list, u);

        for (int64_t j = MAX(y - 1, 0); j <= MIN(y + 1, height - 1); ++j)
        {
            for (int64_t i = MAX(x - 1, 0); i <= MIN(x + 1, width - 1); ++i)
            {
                uint32_t v = j * width + i;

                if (u != v && map[v] == '.' && !is_marked(closed_list, v))
                {
                    double alt = distances[u] + 1 + manhatten(x, y, e_x, e_y); //chebyshev(x, y, e_x, e_y);

                    if (alt < distances[v])
                    {
                        path[v] = u;
                        distances[v] = alt;

                        heap_insert(v);
                    }
                }
            }
        }
    }

    return width * height;
}

int parse_map_metadata(uint32_t* width, uint32_t* height, FILE* fp)
{
    char* line = (char*) malloc(2048);
    char* pos;
    int c, status;
    size_t i;

    i = status = 0;
    while ((c = fgetc(fp)) != EOF)
    {
        if (i >= 2048 || status == 3)
        {
            break;
        }
        else if (c == '\n')
        {
            line[i] = '\0';

            if (i >= 7 && !strncmp("height ", line, 7))
            {
                pos = NULL;
                *height = strtoul(&line[7], &pos, 0);
                if (pos == NULL || *pos != '\0' || *height <= 3)
                {
                    break;
                }
                status |= 2;
            }
            else if (i >= 6 && !strncmp("width ", line, 6))
            {
                pos = NULL;
                *width = strtoul(&line[6], &pos, 0);
                if (pos == NULL || *pos != '\0' || *width <= 3)
                {
                    break;
                }
                status |= 1;
            }

            i = 0; 
        }
        else
        {
            line[i++] = c & 255;
        }
    }

    free(line);
    return status;
}

int read_map(FILE* file)
{
    // Read map
    for (uint32_t i = 0; i < width * height; ++i)
    {
        int c;
        
        while ((c = fgetc(file)) != EOF && c != '.' && c != '@');

        if (c == EOF)
        {
            return 0;
        }

        map[i] = c & 255;
    }

    return 1;
}

void print_path(uint32_t node)
{
    if (path[node] != width * height && path[node] != node)
    {
        print_path(path[node]);
    }
    fprintf(stderr, "(%u,%u)\n", node % width, node / width);
    map[(node / width) * height + node % width] = 'x';
}

int read_int(uint32_t* i, char* str)
{
    char* pos = NULL;
    *i = strtoul(str, &pos, 0);

    if (pos == NULL || *pos != '\0')
    {
        return 0;
    }

    return 1;
}

int main(int argc, char** argv)
{
    if (argc != 6)
    {
        fprintf(stderr, "Usage: %s <map file> <start x> <start y> <end x> <end y>\n", argv[0]);
        return 1;
    }

    FILE* file = fopen(argv[1], "r");
    if (file == NULL)
    {
        fprintf(stderr, "%s is not a valid map file\n", argv[1]);
        return 2;
    }

    if (parse_map_metadata(&width, &height, file) != 3)
    {
        fclose(file);
        fprintf(stderr, "%s is not a valid map file\n", argv[1]);
        return 2;
    }

    map = (char*) malloc(width * height);
    if (map == NULL)
    {
        fclose(file);
        fprintf(stderr, "Out of resources\n");
        return 3;
    }

    if (read_map(file) != 1)
    {
        fclose(file);
        free(map);
        fprintf(stderr, "%s is not a valid map file\n", argv[1]);
        return 2;
    }

    fclose(file);

    uint32_t x, y, end_x, end_y;
    
    if (!read_int(&x, argv[2]) || !read_int(&y, argv[3]) || !read_int(&end_x, argv[4]) || !read_int(&end_y, argv[5]))
    {
        fprintf(stderr, "invalid coordinates\n");
        return 1;
    }

    fprintf(stderr, "Width: %u\nHeight: %u\nStart: (%u, %u)\nEnd: (%u, %u)\n", width, height, x, y, end_x, end_y);

    
    // Find exit points
    map_exits = (uint8_t*) malloc((width * height) >> 3);
    memset(map_exits, 0, (width * height) >> 3);
    mark(map_exits, end_y * width + end_x);

    // Set distance to every node = INFINITY
    distances = (double*) malloc(sizeof(double) * (width * height));
    path = (uint32_t*) malloc(sizeof(uint32_t) * (width * height));
    for (uint32_t i = 0; i < width * height; ++i)
    {
        distances[i] = DBL_MAX;
        path[i] = width * height;
    }


    // Create heap and closed list for A* search
    heap = (uint32_t*) malloc(sizeof(uint32_t) * (width * height));
    heap_size = 0;

    closed_list = (uint8_t*) malloc((width * height) >> 3);
    memset(closed_list, 0, width * height);

    uint32_t exit_node = find_closest_exit(y * width + x, end_x, end_y);
    if (exit_node < width * height)
    {
        fprintf(stderr, "Exit found: (%u,%u)\n", exit_node % width, exit_node / width);

        print_path(exit_node);

        for (uint32_t i = 0; i < width * height; ++i)
        {
            if (i % width == 0 && i > 0)
            {
                printf("\n");
            }
            printf("%c", map[i]);
        }
        printf("\n");
    }
    else
    {
        fprintf(stderr, "No exit found\n");
    }

    free(distances);
    free(heap);
    free(closed_list);
    free(map_exits);
    free(map);
    return 0;
}
