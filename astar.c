#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

char* map = NULL;
uint32_t width = 0, height = 0;

uint8_t* map_exits = NULL;
uint8_t* closed_list = NULL;

uint32_t* distances = NULL;
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


uint32_t find_closest_exit(uint32_t point)
{
    distances[point] = 0;
    path[point] = point;

    heap_insert(point);

    while (heap_size > 0)
    {
        uint32_t u = heap_remove();
        int64_t x = u % width;
        int64_t y = u / height;

        if (is_marked(map_exits, u))
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
                    uint32_t alt = distances[u] + 1;

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

int parse_map_metadata(char* type, size_t len, uint32_t* width, uint32_t* height, FILE* fp)
{
    char* line = (char*) malloc(len);
    char* pos;
    int c, status;
    size_t i;

    i = status = 0;
    while ((c = fgetc(fp)) != EOF)
    {
        if (i >= len || status == 7)
        {
            break;
        }
        else if (c == '\n')
        {
            line[i] = '\0';

            if (i >= 5 && !strncmp("type ", line, 5))
            {
                strncpy(type, &line[5], i - 5);
                status |= 1 << 2;
            }
            else if (i >= 7 && !strncmp("height ", line, 7))
            {
                pos = NULL;
                *height = strtoul(&line[7], &pos, 0);
                if (pos == NULL || *pos != '\0' || *height <= 3)
                {
                    break;
                }
                status |= 1 << 1;
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
            else
            {
                break;
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

void print_path(uint32_t node)
{
    if (path[node] != width * height && path[node] != node)
    {
        print_path(path[node]);
    }
    printf("(%u,%u)\n", node % width, node / width);
}

int main(int argc, char** argv)
{
    if (argc != 4)
    {
        fprintf(stderr, "Usage: %s <map file> <x pos> <y pos>\n", argv[0]);
        return 1;
    }

    FILE* file = fopen(argv[1], "r");
    if (file == NULL)
    {
        fprintf(stderr, "%s is not a valid map file\n", argv[1]);
        return 2;
    }

    uint32_t x, y;

    char* strpos = NULL;
    x = strtoul(argv[2], &strpos, 0);
    if (strpos == NULL || *strpos != '\0')
    {
        fclose(file);
        fprintf(stderr, "%s is not a valid x position\n", argv[2]);
        return 3;
    }

    strpos = NULL;
    y = strtoul(argv[3], &strpos, 0);
    if (strpos == NULL || *strpos != '\0')
    {
        fclose(file);
        fprintf(stderr, "%s is not a valid x position\n", argv[3]);
        return 3;
    }

    char type[256];
    if (parse_map_metadata(type, 256, &width, &height, file) != 7)
    {
        fclose(file);
        fprintf(stderr, "%s is not a valid map file\n", argv[1]);
        return 2;
    }

    if (x >= width || y >= height)
    {
        fclose(file);
        fprintf(stderr, "(%u,%u) is not a valid starting position\n", x, y);
        return 3;
    }

    map = (char*) malloc(width * height);
    if (map == NULL)
    {
        fclose(file);
        fprintf(stderr, "Not enough resources to create %ux%u map\n", width, height);
        return 4;
    }

    // Read map
    for (uint32_t i = 0; i < width * height; ++i)
    {
        int c;
        
        while ((c = fgetc(file)) != EOF && c != '.' && c != '@');
        if (c == EOF)
        {
            fclose(file);
            free(map);
            fprintf(stderr, "%s is not a valid map file\n", argv[1]);
            return 2;
        }

        map[i] = c & 255;
    }

    fclose(file);

    if (map[y * width + x] == '@')
    {
        free(map);
        fprintf(stderr, "(%u,%u) is not a valid starting position\n", x, y);
        return 3;
    }

    fprintf(stderr, "Map: %s\nWidth: %u\nHeight: %u\nPosition: (%u, %u)\n", type, width, height, x, y);

    
    // Find exit points
    map_exits = (uint8_t*) malloc((width * height) >> 3);
    memset(map_exits, 0, (width * height) >> 3);
    uint32_t num_exits = 0;

    // Scan horizontal lines for exits
    for (uint32_t i = 0; i < width; ++i)
    {
        if (map[i] == '.')
        {
            mark(map_exits, i);
            ++num_exits;
        }

        if (map[(height - 1) * width + i] == '.')
        {
            mark(map_exits, (height - 1) * width + i);
            ++num_exits;
        }
    }

    // Scan vertical lines for exits
    for (uint32_t j = 0; j < height; ++j)
    {
        if (map[j * width] == '.')
        {
            mark(map_exits, j * width);
            ++num_exits;
        }

        if (map[j * width + height - 1] == '.')
        {
            mark(map_exits, j * width + height - 1);
            ++num_exits;
        }
    }

    printf("Exits found: %u\n", num_exits);

    // Set distance to every node = INFINITY
    distances = (uint32_t*) malloc(sizeof(uint32_t) * (width * height));
    path = (uint32_t*) malloc(sizeof(uint32_t) * (width * height));
    for (uint32_t i = 0; i < width * height; ++i)
    {
        distances[i] = UINT_MAX;
        path[i] = width * height;
    }


    // Create heap and closed list for A* search
    heap = (uint32_t*) malloc(sizeof(uint32_t) * (width * height));
    heap_size = 0;

    closed_list = (uint8_t*) malloc((width * height) >> 3);
    memset(closed_list, 0, width * height);

    uint32_t exit_node = find_closest_exit(y * width + x);
    if (exit_node < width * height)
    {
        fprintf(stderr, "Exit found: (%u,%u)\n", exit_node % width, exit_node / width);

        print_path(exit_node);
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
