/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/**************************************************************************
 * Desc: Global map storage functions
 * Author: Andrew Howard
 * Date: 6 Feb 2003
 * CVS: $Id: map_store.c 2951 2005-08-19 00:48:20Z gerkey $
**************************************************************************/

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "amcl/map/map.h"


////////////////////////////////////////////////////////////////////////////
// Load an occupancy grid
int map_load_occ(map_t *map, const char *filename, double scale, int negate)
{
  FILE *file;
  char magic[3];
  int i, j;
  int ch, occ;
  int width, height, depth;
  map_cell_t *cell;

  // Open file
  file = fopen(filename, "r");
  if (file == NULL)
  {
    fprintf(stderr, "%s: %s\n", strerror(errno), filename);
    return -1;
  }
  int tt=0;
  FILE *file2 = fopen("output.txt", "a"); // 使用 "a" 模式以追加的方式打开文件
  if (file2 == NULL) {
            printf("打开文件失败\n");
            return; // 如果文件无法打开，则退出函数
  }
  // Read ppm header
  
  if ((fscanf(file, "%2s \n", magic) != 1) || (strcmp(magic, "P5") != 0))
  {
    fprintf(stderr, "incorrect image format; must be PGM/binary");
    fclose(file);
    return -1;
  }

  // Ignore comments
  while ((ch = fgetc(file)) == '#')
    while (fgetc(file) != '\n');
  ungetc(ch, file);

  // Read image dimensions
  if(fscanf(file, " %d %d \n %d \n", &width, &height, &depth) != 3)
  {
    fprintf(stderr, "Failed ot read image dimensions");
    return -1;
  }
    printf("width: %d, height: %d, depth: %d\n", width, height, depth);
  // Allocate space in the map
  if (map->cells == NULL)
  {
    map->scale = scale;
    map->size_x = width;
    map->size_y = height;
    map->cells = calloc(width * height, sizeof(map->cells[0]));
  }
  else
  {
    if (width != map->size_x || height != map->size_y)
    {
      //PLAYER_ERROR("map dimensions are inconsistent with prior map dimensions");
      return -1;
    }
  }
  
  // Read in the image
  for (j = 0; j <height; j++)
  {
    for (i = 0; i < width; i++)
    {
      ch = fgetc(file);
      
      // Black-on-white images
      if (!negate)
      {
        if (ch < 100){
          tt++;
          occ = +1;
        } 
        else if (ch >= depth-1){
          fprintf(file2, "j: %d i:%d\n", j, i);
          
          printf("j: %d i:%d\n", j,i);
          occ = -1;
          tt--;
        }
        else{
          occ = 0;
          tt++;
        }
          
      }

      // White-on-black images
      else
      {
        if (ch < depth / 4)
          occ = -1;
        else if (ch > 3 * depth / 4)
          occ = +1;
        else
          occ = 0;
      }
      
      if (!MAP_VALID(map, i, j))
        continue;
      cell = map->cells + MAP_INDEX(map, i, j);
      cell->occ_state = occ;
    }
  }
  fclose(file2);
  printf("tt: %d\n", tt);
  fclose(file);
  FILE *pgmFile = fopen("output.pgm", "w");
  if (pgmFile == NULL) {
      fprintf(stderr, "Cannot open file to write\n");
      exit(1);
  }

  // 写入PGM头部
  fprintf(pgmFile, "P2\n%d %d\n255\n", width, height);

  // 遍历图像数据并写入文件
  for (int j = 0; j < height; j++) {
      for (int i = 0; i < width; i++) {
          map_cell_t *cell = map->cells + MAP_INDEX(map, i, j);
          int pixelValue = (cell->occ_state == -1) ? 0 : 255; // occ为-1输出0，否则输出255
          fprintf(pgmFile, "%d ", pixelValue);
      }
      fprintf(pgmFile, "\n");
  }

  // 关闭文件
  fclose(pgmFile);
  return 0;
}


////////////////////////////////////////////////////////////////////////////
// Load a wifi signal strength map
/*
int map_load_wifi(map_t *map, const char *filename, int index)
{
  FILE *file;
  char magic[3];
  int i, j;
  int ch, level;
  int width, height, depth;
  map_cell_t *cell;

  // Open file
  file = fopen(filename, "r");
  if (file == NULL)
  {
    fprintf(stderr, "%s: %s\n", strerror(errno), filename);
    return -1;
  }

  // Read ppm header
  fscanf(file, "%10s \n", magic);
  if (strcmp(magic, "P5") != 0)
  {
    fprintf(stderr, "incorrect image format; must be PGM/binary");
    return -1;
  }

  // Ignore comments
  while ((ch = fgetc(file)) == '#')
    while (fgetc(file) != '\n');
  ungetc(ch, file);

  // Read image dimensions
  fscanf(file, " %d %d \n %d \n", &width, &height, &depth);

  // Allocate space in the map
  if (map->cells == NULL)
  {
    map->size_x = width;
    map->size_y = height;
    map->cells = calloc(width * height, sizeof(map->cells[0]));
  }
  else
  {
    if (width != map->size_x || height != map->size_y)
    {
      //PLAYER_ERROR("map dimensions are inconsistent with prior map dimensions");
      return -1;
    }
  }

  // Read in the image
  for (j = height - 1; j >= 0; j--)
  {
    for (i = 0; i < width; i++)
    {
      ch = fgetc(file);

      if (!MAP_VALID(map, i, j))
        continue;

      if (ch == 0)
        level = 0;
      else
        level = ch * 100 / 255 - 100;

      cell = map->cells + MAP_INDEX(map, i, j);
      cell->wifi_levels[index] = level;
    }
  }
  
  fclose(file);

  return 0;
}
*/



