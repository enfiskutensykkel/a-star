#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define D 8.0 // increase this to make A* run faster

#define is_marked(bitmap, point) \
    (!!((bitmap)[(point) >> 3] & (1 << ((point) & 7))))

#define mark(bitmap, point) \
    ((bitmap)[(point) >> 3] |= 1 << ((point) & 7))



void set_cost_lut(double* cost_lut, uint32_t infinity)
{
    for (int i = 0; i < 256; ++i)
    {
        cost_lut[i] = infinity;
    }
    cost_lut['T'] = 5.0;
    cost_lut['@'] = infinity;
    cost_lut['.'] = 1.0;
}



int read_map(FILE* file, uint8_t* map, uint32_t width, uint32_t height)
{
    for (uint32_t i = 0, n = width * height; i < n; ++i)
    {
        int c;

        while ((c = fgetc(file)) != EOF && (c < 0x20 || c > 0x7e));

        if (c == EOF)
        {
            return 0;
        }

        map[i] = c & 255;
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



void print_path(uint8_t* map, uint32_t width, uint32_t height, uint32_t* rev_path, uint32_t point)
{
    if (rev_path[point] != point && rev_path[point] < width * height)
    {
        print_path(map, width, height, rev_path, rev_path[point]);
    }
    fprintf(stdout, "(%u,%u)\n", point % width, point / height);

    map[(point / height) * width + (point % width)] = 'X';
}



void print_map(const uint8_t* map, uint32_t width, uint32_t height)
{
    for (uint32_t y = 0; y < height; ++y)
    {
        for (uint32_t x = 0; x < width; ++x)
        {
            fprintf(stderr, "%c", map[y * width + x]);
        }
        fprintf(stderr, "\n");
    }
}



inline void heap_insert(uint32_t* heap, uint32_t* heap_size, double* fcosts, uint32_t point)
{
    uint32_t hole;

    for (hole = ++*heap_size; hole > 1 && fcosts[point] < fcosts[heap[hole >> 1]]; hole >>= 1)
    {
        heap[hole] = heap[hole >> 1];
    }

    heap[hole] = point;
}



inline uint32_t heap_remove(uint32_t* heap, uint32_t* heap_size, double* fcosts)
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



inline double euclidean(uint32_t ux, uint32_t uy, uint32_t vx, uint32_t vy)
{
    uint32_t dx, dy;

    dx = (ux >= vx) ? ux - vx : vx - ux;
    dy = (uy >= vy) ? uy - vy : vy - uy;

    return D * sqrt(dx * dx + dy * dy);
}



inline double diagonal(uint32_t ux, uint32_t uy, uint32_t vx, uint32_t vy)
{
    uint32_t dx, dy;

    dx = (ux >= vx) ? ux - vx : vx - ux;
    dy = (uy >= vy) ? uy - vy : vy - uy;

    // octile distance... for chebyshev, change D2 to sqrt(2)
    double D2 = 1.0;

    return D * (dx + dy) + (D2 - 2.0 * D) * (dx < dy ? dx : dy);
}


/*
inline double manhattan(uint32_t ux, uint32_t uy, uint32_t vx, uint32_t vy)
{
    uint32_t dx, dy;

    dx = (ux >= vx) ? ux - vx : vx - ux;
    dy = (uy >= vy) ? uy - vy : vy - uy;

    return D * (dx + dy);
}
*/


uint32_t search(
        const uint8_t* map, uint32_t width, uint32_t height, 
        double* cost_lut, uint32_t* rev_path, double* f_costs, double* g_costs,
        uint32_t* open_list, uint8_t* closed_list,
        uint32_t start, uint32_t target
        )
{
    // Initialize closed list and cost table and reverse path
    for (uint32_t i = 0, n = width * height; i < n; ++i)
    {
        closed_list[i >> 3] = 0;
        rev_path[i] = i; // set shortest path through self
        g_costs[i] = f_costs[i] = width * height + 1.0; // set distance to infinity
    }

    uint32_t open_list_size = 0;

    // Start off with start node
    f_costs[start] = 0.0;
    g_costs[start] = 0.0;
    heap_insert(open_list, &open_list_size, f_costs, start);

    // Get target coordinates
#ifndef DIJKSTRA
    uint32_t tx = target % width;
    uint32_t ty = target / height;
#endif

    while (open_list_size > 0)
    {
        // remove from open list the point u with smallest f_cost
        uint32_t u = heap_remove(open_list, &open_list_size, f_costs);
        uint32_t ux = u % width;
        uint32_t uy = u / height;

        // check if u is our target
        if (u == target)
        {
            return u;
        }

        // insert u into closed list
#ifndef DIJKSTRA
        mark(closed_list, u);
#endif

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

                // skip self
                if (u == v)
                {
                    continue;
                }

#ifndef DIJKSTRA
                // skip marked nodes
                if (is_marked(closed_list, v))
                {
                    continue;
                }
#endif

                // calculate tentative g score
                double alt = g_costs[u] + cost_lut[map[v]] + euclidean(ux, uy, vx, vy);

                // use u to reach v if path to v is shorter through u
                if (alt < g_costs[v])
                {
                    rev_path[v] = u;
                    g_costs[v] = alt;
#ifndef DIJKSTRA
                    f_costs[v] = alt + diagonal(vx, vy, tx, ty);
#else
                    f_costs[v] = alt;
#endif

                    heap_insert(open_list, &open_list_size, f_costs, v);
                }
            }
        }
    }

    // return an illegal point to indicate that no path was found
    return width * height;
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

    // Allocate resources
    size_t size = width * height;
    void* mem = malloc(
            sizeof(uint8_t) * size +        // map
            sizeof(double) * 256 +          // object cost lookup table
            sizeof(uint32_t) * size +       // reverse path routing table
            sizeof(double) * size +         // f_cost table
            sizeof(double) * size +         // g_cost table
            sizeof(uint32_t) * (size + 1) + // open list
            sizeof(uint8_t) * (size >> 3)   // closed list
            );
    if (mem == NULL)
    {
        fprintf(stderr, "Not enough resources\n");
        return 1;
    }
    uint8_t* map = (uint8_t*) mem;
    double* cost_lut = (double*) (map + size);
    uint32_t* path_tbl = (uint32_t*) (cost_lut + 256);
    double* f_costs = (double*) (path_tbl + size);
    double* g_costs = (double*) (f_costs + size);
    uint32_t* olist = (uint32_t*) (g_costs + size);
    uint8_t* clist = (uint8_t*) (olist + size + 1);

    // Parse map file
    if (!read_map(file, map, width, height))
    {
        free(mem);
        fclose(file);
        fprintf(stderr, "Couldn't parse map '%s'\n", argv[5]);
        return 1;
    }

    fclose(file);

    // Make a cost look-up table
    set_cost_lut(cost_lut, width * height + 1);

    // Calculate start coordinates and target coordinates
    uint32_t start = coordinates[1] * width + coordinates[0];
    uint32_t target = coordinates[3] * width + coordinates[2];

    // Calculate shortest path to target point
    uint32_t exit_point = search(map, width, height, cost_lut, path_tbl, f_costs, g_costs, olist, clist, start, target);

    if (exit_point == target)
    {
        print_path(map, width, height, path_tbl, exit_point);
        print_map(map, width, height);
    }
    else
    {
        printf("No path found\n");
    }

    free(mem);

    return 0;
}
