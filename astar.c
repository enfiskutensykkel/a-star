#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>



int read_map(FILE* file, char* map, uint64_t* objects, uint32_t width, uint32_t height)
{
    for (int i = 0; i < 256; ++i)
    {
        objects[i] = 0;
    }

    for (uint32_t i = 0, n = width * height; i < n; ++i)
    {
        int c;

        while ((c = fgetc(file)) != EOF && (c < 0x20 || c > 0x7e));

        if (c == EOF)
        {
            return 0;
        }

        map[i] = c & 255;
        ++objects[c & 255];
    }

    return 1;
}



int read_metadata(FILE* file, uint32_t* width, uint32_t* height)
{
    char line[2048];
    char* pos;
    
    int status = 0;

    for (int i = 0; i < 4 && !feof(file); ++i)
    {
        fgets(line, sizeof(line), file);

        for (size_t j = 0; j < sizeof(line); ++j)
        {
            if (line[j] < 0x20 || line[j] > 0x7e)
            {
                line[j] = '\0';
                break;
            }
        }

        if (strncmp("width ", line, 6) == 0)
        {
            pos = NULL;
            *width = strtoul(&line[6], &pos, 0);
            if (pos == NULL || *pos != '\0')
            {
                return 0;
            }
            status |= 1;
        }
        else if (strncmp("height ", line, 7) == 0)
        {
            pos = NULL;
            *height = strtoul(&line[7], &pos, 0);
            if (pos == NULL || *pos != '\0')
            {
                return 0;
            }
            status |= 2;
        }
    }

    return !!(status & 3);
}



void print_path(char* map, uint32_t width, uint32_t height, uint32_t* rev_path, uint32_t point)
{
    if (rev_path[point] != point && rev_path[point] < width * height)
    {
        print_path(map, width, height, rev_path, rev_path[point]);
    }
    fprintf(stdout, "(%u,%u)\n", point % width, point / height);
    map[(point / height) * width + (point % width)] = 'x';
}



void print_map(char* map, uint32_t width, uint32_t height)
{
    for (uint32_t y = 0; y < width; ++y)
    {
        for (uint32_t x = 0; x < width; ++x)
        {
            fprintf(stderr, "%c", map[y * width + x]);
        }
        fprintf(stderr, "\n");
    }
}



void heap_insert(uint32_t* heap, uint32_t* heap_size, double* fcosts, uint32_t point)
{
    uint32_t hole;

    for (hole = ++*heap_size; hole > 1 && fcosts[point] < fcosts[heap[hole >> 1]]; hole >>= 1)
    {
        heap[hole] = heap[hole >> 1];
    }

    heap[hole] = point;
}



uint32_t heap_remove(uint32_t* heap, uint32_t* heap_size, double* fcosts)
{
    uint32_t removed_point = heap[1];

    uint32_t point = heap[1] = heap[(*heap_size)--];
    uint32_t hole, child;

    for (hole = 1; (hole << 1) <= *heap_size; hole = child)
    {
        child = hole << 1;

        if (child != *heap_size && fcosts[heap[child + 1]] < fcosts[heap[child]])
        {
            ++child;
        }

        if (fcosts[heap[child]] < fcosts[point])
        {
            heap[hole] = heap[child];
        }
        else
        {
            break;
        }
    }

    heap[hole] = point;

    return removed_point;
}



int is_marked(uint8_t* bitmap, uint32_t point)
{
    return !!(bitmap[point >> 3] & (1 << (point & 7)));
}



void mark(uint8_t* bitmap, uint32_t point)
{
    bitmap[point >> 3] |= 1 << (point & 7);
}



double manhatten(uint32_t ux, uint32_t uy, uint32_t vx, uint32_t vy)
{
    uint64_t tx, ty;

    tx = (ux >= vx) ? ux - vx : vx - ux;
    ty = (uy >= vy) ? uy - vy : vy - uy;

    return 2.0 * (tx + ty);
}



uint32_t search(char* map, uint32_t width, uint32_t height, double* cost_lut, uint32_t* rev_path, uint32_t start, uint32_t target)
{
    // Allocate resources
    double* f_costs = (double*) malloc(sizeof(double) * width * height);
    uint32_t* g_costs = (uint32_t*) malloc(sizeof(uint32_t) * width * height);
    uint8_t* closed_list = (uint8_t*) malloc(width * height);

    // Initialize closed list and cost table and reverse path
    for (uint32_t i = 0, n = width * height; i < n; ++i)
    {
        closed_list[i >> 3] = 0;
        f_costs[i] = width * height; // infinity
        g_costs[i] = width * height;
        rev_path[i] = i;
    }

    // Initialize open list
    uint32_t* open_list = (uint32_t*) malloc(sizeof(uint32_t) * width * height);
    uint32_t open_list_size = 0;

    uint32_t found = width * height;

    // Initalize with start node
    f_costs[start] = 0.0;
    g_costs[start] = 0;
    heap_insert(open_list, &open_list_size, f_costs, start);

    while (open_list_size > 0)
    {
        // remove from open list the point u with smallest f_cost
        uint32_t u = heap_remove(open_list, &open_list_size, f_costs);
        uint32_t ux = u % width;
        uint32_t uy = u / height;

        if (u == target)
        {
            found = u;
            break;
        }

        // insert u into closed list
        mark(closed_list, u);

        // do some boundary checking magic
        uint32_t vx_lo = ux > 0 ? ux - 1 : 0;
        uint32_t vx_hi = ux < width - 1 ? ux + 1 : width - 1;
        uint32_t vy_lo = uy > 0 ? uy - 1: 0;
        uint32_t vy_hi = uy < height - 1 ? uy + 1 : height - 1;

        // for neighbour v of u:
        for (uint32_t vy = vy_lo; vy <= vy_hi; ++vy)
        {
            for (uint32_t vx = vx_lo; vx <= vx_hi; ++vx)
            {
                uint32_t v = vy * width + vx;

                // if v is not marked
                if (u == v && !is_marked(closed_list, v))
                {
                    continue;
                }

                // calculate tentative g score
                uint32_t alt = g_costs[u] + 1;

                // use u to reach v if path to v is shorter through u
                if (alt < g_costs[v])
                {
                    rev_path[v] = u;
                    g_costs[v] = alt;
                    f_costs[v] = alt + cost_lut[map[v]] + manhatten(ux, uy, vx, vy);

                    heap_insert(open_list, &open_list_size, f_costs, v);
                }
            }
        }
    }

    free(open_list);
    free(closed_list);
    free(f_costs);
    free(g_costs);
    return found;
}



int main(int argc, char** argv)
{
    if (argc < 6)
    {
        fprintf(stderr, "Usage: %s <start-x> <start-y> <target-x> <target-y> <map-file>\n", argv[0]);
        return 1;
    }

    // Parse coordinates
    long coordinates[4] = {-1, -1, -1, -1};

    for (int i = 1; i < argc && i < 5; ++i)
    {
        char* pos = NULL;
        coordinates[i - 1] = strtol(argv[i], &pos, 0);

        if (pos == NULL || *pos != '\0' || coordinates[i-1] < 0)
        {
            fprintf(stderr, "'%s' is not a valid map coordinate\n", argv[i]);
            return 1;
        }
    }

    // Parse map file
    FILE* file = fopen(argv[5], "r");
    if (file == NULL)
    {
        fprintf(stderr, "'%s' is not a valid map\n", argv[5]);
        return 1;
    }

    // Extract map metadata (width and height) from file
    uint32_t width, height;
    if (!read_metadata(file, &width, &height))
    {
        fclose(file);
        fprintf(stderr, "'%s' is not a valid map\n", argv[5]);
        return 1;
    }

    // Check that given coordinates are legal
    if (coordinates[0] >= width || coordinates[1] >= height)
    {
        fclose(file);
        fprintf(stderr, "(%ld,%ld) is not a point in the map!\n", coordinates[0], coordinates[1]);
        return 1;
    }
    else if (coordinates[0] >= width || coordinates[1] >= height)
    {
        fclose(file);
        fprintf(stderr, "(%ld,%ld) is not a point in the map!\n", coordinates[0], coordinates[1]);
        return 1;
    }

    char* map = (char*) malloc(width * height);
    uint64_t objects[256];

    // Parse map file
    if (!read_map(file, map, objects, width, height))
    {
        free(map);
        fclose(file);
        fprintf(stderr, "Couldn't parse map '%s'\n", argv[5]);
        return 1;
    }

    fclose(file);

    uint32_t sx, sy, tx, ty;
    sx = (uint32_t) coordinates[0];
    sy = (uint32_t) coordinates[1];
    tx = (uint32_t) coordinates[2];
    ty = (uint32_t) coordinates[3];

    // Print some info
//    fprintf(stderr, "Map            : %s (%u x %u)\n", argv[5], width, height);
//    fprintf(stderr, "Objects in map :");
//    for (int i = 0; i < 256; ++i)
//    {
//        if (objects[i] > 0)
//        {
//            fprintf(stderr, " %c", i);
//        }
//    }
//    fprintf(stderr, "\n");
//    fprintf(stderr, "Starting coords: (%u,%u)\n", sx, sy);
//    fprintf(stderr, "Target coords  : (%u,%u)\n", tx, ty);

    
    // Make a cost look-up table
    double cost_lut[256];
    cost_lut['@'] = (width * height) * 2.0; // set cost of walls to infinity
    cost_lut['.'] = 1.0;

    // Make routing table
    uint32_t* reverse_path = (uint32_t*) malloc(sizeof(uint32_t) * width * height);

    uint32_t s = sy * width + sx;
    uint32_t t = ty * width + tx;

    uint32_t exit_point = search(map, width, height, cost_lut, reverse_path, s, t);

    if (exit_point == t)
    {
        print_path(map, width, height, reverse_path, exit_point);

        print_map(map, width, height);
    }
    else
    {
        printf("No path found\n");
    }

    free(reverse_path);
    free(map);

    return 0;
}
