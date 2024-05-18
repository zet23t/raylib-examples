/*******************************************************************************************
*
*   raylib [sdf] example - SDF Testbed
*
*   Example originally created with raylib 5.0
*
*   Example contributed by Eike Decker (@zet23t) and reviewed by Ramon Santamaria (@raysan5)
*
*   Example licensed under an unmodified zlib/libpng license, which is an OSI-certified,
*   BSD-like license that allows static linking with closed source software
*
*   Copyright (c) 2019-2024 Eike Decker (@zet23t ) and Ramon Santamaria (@raysan5)
*
********************************************************************************************
*
*   This example demonstrates how to use signed distance fields (SDF) for improving
*   pathfinding.
*
*   1) Unit size: A path may have a requirement for a minimum width to allow passage.
*      By using the SDF values, we can block paths that are too narrow for the unit.
*
*   2) Path preferences: A unit may prefer to stay close to walls or avoid them.
*      The example demonstrates how to influence the pathfinding by using the SDF values.
*
*   3) Varying step distances: Using SDF values to adjust step distances during path 
*      finding, resulting in curved paths.
*
*  TODO: optimize the path after it was found: nodes could be removed to smooth the path
*        this could again be done using the SDF values to determine if a node can be removed
*        without clipping through walls
*
********************************************************************************************/

#include "raylib.h"
#include "raymath.h"

#include <stddef.h> // Required for: NULL

typedef struct PathfindingNode
{
    int x, y;
    int fromX;
    int fromY;
    int score;
} PathfindingNode;

typedef struct NeighborOffset
{
    int x, y;
    int distance;
} NeighborOffset;

// a simple cat face that can be drawn as a triangle fan
Vector2 catFace[] = {
    {0.0f, 1.0f},
    {0.7f, 1.0f},
    {1.0f, 0.7f},
    {1.0f, -1.0f},
    {0.5f, -0.6f},
    {-0.5f, -0.6f},
    {-1.0f, -1.0f},
    {-1.0f, 0.7f},
    {-0.7f, 1.0f},
};

// a simple rat face that can be drawn as a triangle fan
Vector2 ratFace[] = {
    {0.0f, 1.0f},
    {0.3f, 0.9f},
    {0.8f, -0.2f},
    {1.0f, -0.8f},
    {0.8f, -1.0f},
    {0.5f, -1.0f},
    {0.3f, -0.7f},
    {0.0f, -0.8f},
    {-0.3f, -0.7f},
    {-0.5f, -1.0f},
    {-0.8f, -1.0f},
    {-1.0f, -0.8f},
    {-0.8f, -0.2f},
    {-0.3f, 0.9f},
};

const int gridWidth = 80;
const int gridHeight = 45;
const int cellSize = 10;
char* blockedCells;
char* sdfCells;

// lookup table for cheap square root calculation
int isqrt[256];

// various offsets and distances for jumping nodes during pathfinding
NeighborOffset neighborOffsets[20*20];
int neighborOffsetCount;

void FindPath(PathfindingNode *map, int toX, int toY, int startX, int startY, int unitSize, int *pathLength, PathfindingNode *path, int sdfFactor, int enableJumping)
{
    // the queue should in theory not be longer than the number of map cells
    PathfindingNode *queue = (PathfindingNode *)MemAlloc(gridWidth * gridHeight * sizeof(PathfindingNode));
    for (int i = 0; i < gridWidth * gridHeight; i++)
    {
        map[i].score = 0;
    }

    // initialize queue and map with start position data
    int queueLength = 1;
    queue[0].fromX = startX;
    queue[0].fromY = startY;
    queue[0].x = startX;
    queue[0].y = startY;
    queue[0].score = 1;
    map[startY * gridWidth + startX].fromX = -1;
    map[startY * gridWidth + startX].fromY = -1;
    map[startY * gridWidth + startX].x = startX;
    map[startY * gridWidth + startX].y = startY;
    map[startY * gridWidth + startX].score = 1;

    while (queueLength > 0)
    {
        // find and dequeue node with lowest score
        int lowestScoreIndex = 0;
        for (int i=1;i<queueLength;i++)
        {
            if (queue[i].score < queue[lowestScoreIndex].score)
            {
                lowestScoreIndex = i;
            }
        }
        PathfindingNode node = queue[lowestScoreIndex];
        for (int i=lowestScoreIndex+1;i<queueLength;i++)
        {
            queue[i-1] = queue[i];
        }
        queueLength--;

        // we can determine how far we can safely jump away from this cell by 
        // taking the SDF value of the current cell. If our unit size is 2 and 
        // the SDF value is 5, we can safely jump 3 cells away from this cell, knowing
        // that we can't clip through walls at this distance.
        int cellSdf = sdfCells[node.y * gridWidth + node.x];
        int maxDistance = cellSdf - unitSize;
        if (maxDistance < 1)
        {
            maxDistance = 1;
        }

        // The neighbor offsets are used to check various directions of different distances
        for (int i=0; i<neighborOffsetCount; i++)
        {
            // step distance is the distance the offset is away from the current cell
            // if it exceeds the max distance, we skip this offset
            int stepDistance = neighborOffsets[i].distance;
            if (stepDistance > maxDistance || (!enableJumping && stepDistance > 1))
            {
                continue;
            }

            // rejecting first cells that are outside the map
            int x = node.x + neighborOffsets[i].x;
            int y = node.y + neighborOffsets[i].y;
            if (x < 0 || x >= gridWidth || y < 0 || y >= gridHeight)
            {
                continue;
            }

            // nextSdf is the SDF value of the next cell where we would land
            int nextSdf = sdfCells[y * gridWidth + x];

            // skip if the next cell is closer to a wall than the unit size (wall clipping)
            if (nextSdf < unitSize)
            {
                continue;
            }

            // calculate the score of the next cell
            int score = node.score + stepDistance;
            int sdfValue = sdfCells[y * gridWidth + x];
            // assuming a linear interpolation between the SDF values of the current and next cell,
            // we can estimate the integral of the SDF values between the two cells - this is
            // only a rough approximation and since it's integers, we cheat a bit to favor longer jumps
            int integratedSdfValue = (sdfValue + cellSdf) * (stepDistance + 1) / 2;
            score = score + integratedSdfValue * sdfFactor / 6;

            // if the cell is not yet visited or the score is lower than the previous score, 
            // we update the cell and queue the cell for evaluation (one optimization would be to
            // not queue the cell if it is already queued, but this complexity is omitted here)
            if (map[y * gridWidth + x].score == 0 || score < map[y * gridWidth + x].score)
            {
                map[y * gridWidth + x] = (PathfindingNode){
                    .fromX = node.x,
                    .fromY = node.y,
                    .x = x,
                    .y = y,
                    .score = score
                };
                // queue the cell for evaluation
                queue[queueLength] = map[y * gridWidth + x];
                queueLength++;

                // prevent queue overflow - should not happen with the chosen queue lengths, but
                // could (maybe) still happen for extreme worst case scenarios
                if (queueLength >= gridWidth * gridHeight)
                {
                    TraceLog(LOG_ERROR, "queue overflow\n");
                    queueLength = gridWidth * gridHeight - 1;
                }
            }
        }
    }

    if (map[toY * gridWidth + toX].score > 0)
    {
        // path found
        int x = toX;
        int y = toY;
        int length = 0;
        // reconstruct path by following the from pointers to previous cells - the list is reversed
        // but we handle this with swapping the start / end points
        while (map[y * gridWidth + x].score > 0 && (x != startX || y != startY) && length < gridWidth * gridHeight)
        {
            path[length] = map[y * gridWidth + x];
            x = path[length].fromX;
            y = path[length].fromY;
            length++;
        }
        path[length++] = map[startY * gridWidth + startX];
        
        *pathLength = length;
    }
    else
    {
        // no path found
        *pathLength = 0;
    }

    MemFree(queue);
}

void DrawPathMovement(PathfindingNode* path, int pathCount, float *walkedPathDistance, float movementSpeed, int radius, Color color, Vector2 *points, int pointCount)
{
    if (pathCount == 0)
    {
        return;
    }

    float dt = GetFrameTime();
    *walkedPathDistance += dt * movementSpeed;
    float pointDistance = 0.0f;
    for (int i=1;i<pathCount;i++)
    {
        PathfindingNode p1 = path[i-1];
        PathfindingNode p2 = path[i];
        float dx = p2.x - p1.x;
        float dy = p2.y - p1.y;
        float d = sqrtf(dx * dx + dy * dy);
        if (pointDistance + d >= *walkedPathDistance)
        {
            float t = (*walkedPathDistance - pointDistance) / d;
            float x = p1.x + dx * t + 0.5f;
            float y = p1.y + dy * t + 0.5f;
            if (points != NULL && pointCount > 0)
            {
                Vector2 fan[32];
                for (int j=0;j<pointCount;j++)
                {
                    float px = points[j].x * radius + x * cellSize;
                    float py = points[j].y * radius + y * cellSize;
                    fan[j] = (Vector2){ px, py };
                }
                DrawTriangleFan(fan, pointCount, color);
            }
            else
                DrawCircle(x * cellSize, y * cellSize, radius, color);
            
            return;
        }
        pointDistance += d;
    }
    *walkedPathDistance = 0.0f;
} 

float CalcPathLength(PathfindingNode* path, int pathCount)
{
    float length = 0.0f;
    for (int i=1;i<pathCount;i++)
    {
        PathfindingNode p1 = path[i-1];
        PathfindingNode p2 = path[i];
        float dx = p2.x - p1.x;
        float dy = p2.y - p1.y;
        length += sqrtf(dx * dx + dy * dy);
    }
    return length;
}

//------------------------------------------------------------------------------------
// Program main entry point
//------------------------------------------------------------------------------------
int main(void)
{
    SetTraceLogLevel(LOG_ALL);
    // Initialization
    //--------------------------------------------------------------------------------------
    const int screenWidth = 800;
    const int screenHeight = 450;

    InitWindow(screenWidth, screenHeight, "raylib [sdf pathfinding] example");

    SetTargetFPS(60);

    // initialize square root lookup table for cheap square root calculation
    for (int i=0;i<256;i++)
    {
        isqrt[i] = (int)ceilf(sqrtf(i));
    }

    for (int x = -10; x <= 10; x++)
    {
        for (int y = -10; y <= 10; y++)
        {
            int d = isqrt[x * x + y * y];
            if (d <= 10 && d > 0)
            {
                neighborOffsets[neighborOffsetCount] = (NeighborOffset){ x, y, d };
                neighborOffsetCount++;
            }
        }
    }

    const Color gridColor = (Color){ 200, 200, 200, 40 };
    const Color cellHighlightColor = (Color){ 200, 0, 0, 80 };
    const float movementSpeed = 3.0f;

    blockedCells = (char *)MemAlloc(gridWidth * gridHeight * sizeof(char));
    sdfCells = (char *)MemAlloc(gridWidth * gridHeight * sizeof(char));

    // rat setup
    int pathRatStartX = 5;
    int pathRatStartY = 25;
    int pathRatEndX = 75;
    int pathRatEndY = 25;
    PathfindingNode *mapRat = (PathfindingNode *)MemAlloc(gridWidth * gridHeight * sizeof(PathfindingNode));
    PathfindingNode *pathRat = (PathfindingNode *)MemAlloc(gridWidth * gridHeight * sizeof(PathfindingNode) * 4);
    int pathRatLength = 0;
    int ratWallFactor = 2;

    // cat setup
    int pathCatStartX = 5;
    int pathCatStartY = 25;
    int pathCatEndX = 75;
    int pathCatEndY = 25;
    PathfindingNode *mapCat = (PathfindingNode *)MemAlloc(gridWidth * gridHeight * sizeof(PathfindingNode));
    PathfindingNode *pathCat = (PathfindingNode *)MemAlloc(gridWidth * gridHeight * sizeof(PathfindingNode) * 4);
    int pathCatLength = 0;

    float walkedPathDistanceRat = 0.0f;
    float walkedPathDistanceCat = 0.0f;

    // toggle flags for mouse and keyboard input
    int visualizeMode = 0;
    int randomizeBlocks = 1;
    char paintMode = 0;
    int updateSDF = 1;
    int sdfFunction = 0;
    int jumpingEnabled = 1;
    //--------------------------------------------------------------------------------------

    // Main game loop
    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        //----------------------------------------------------------------------------------
        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

            ClearBackground((Color) {170,200,150,255});

            Vector2 mousePos = GetMousePosition();
            int cellX = mousePos.x / cellSize;
            int cellY = mousePos.y / cellSize;

            //----------------------------------------------------------------------------------
            // mouse input handling
            //----------------------------------------------------------------------------------
            if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON))
            {
                paintMode = blockedCells[cellY * gridWidth + cellX] == 1 ? 0 : 1;
            }
            if (IsMouseButtonDown(MOUSE_LEFT_BUTTON))
            {
                blockedCells[cellY * gridWidth + cellX] = paintMode;
                updateSDF = 1;
            }

            //----------------------------------------------------------------------------------
            // keyboard input handling
            //----------------------------------------------------------------------------------
            if (IsKeyDown(KEY_C)) 
            {
                for (int i = 0; i < gridWidth * gridHeight; i++)
                {
                    blockedCells[i] = 0;
                }
                updateSDF = 1;
            }

            if (IsKeyPressed(KEY_V))
            {
                visualizeMode++;
            }

            if (IsKeyPressed(KEY_Q))
            {
                ratWallFactor = (ratWallFactor + 1) % 8;
                updateSDF = 1;
            }

            if (IsKeyPressed(KEY_R))
            {
                randomizeBlocks = 1;
            }

            if (IsKeyPressed(KEY_S))
            {
                sdfFunction = (sdfFunction + 1) % 3;
                updateSDF = 1;
            }

            if (IsKeyPressed(KEY_J))
            {
                jumpingEnabled = !jumpingEnabled;
                updateSDF = 1;
            }

            //----------------------------------------------------------------------------------
            // initialize map with random blocks
            //----------------------------------------------------------------------------------
            if (randomizeBlocks)
            {
                randomizeBlocks = 0;
                updateSDF = 1;
                for (int i = 0; i < gridWidth * gridHeight; i++)
                {
                    blockedCells[i] = 0;
                }
                for (int i=0;i<40;i++)
                {
                    int x = GetRandomValue(15, gridWidth-15);
                    int y = GetRandomValue(15, gridHeight-15);
                    int s = GetRandomValue(1, 2);
                    int v = GetRandomValue(0, 1);
                    for (int j=-s;j<=s;j++)
                    {
                        for (int k=-s;k<=s;k++)
                        {
                            blockedCells[(y+j)*gridWidth + x+k] = v;
                        }
                    }
                }
            }

            //----------------------------------------------------------------------------------
            // update sdf values and execute pathfinding
            //----------------------------------------------------------------------------------
            if (updateSDF)
            {
                updateSDF = 0;
                // calculate sdf values
                // update sdf values to max distance we want to consider
                for (int i = 0; i < gridWidth * gridHeight; i++)
                {
                    sdfCells[i] = 10;
                }
                for (int y = 0; y < gridHeight; y++)
                {
                    for (int x = 0; x < gridWidth; x++)
                    {
                        // for each cell, we update the surrounding cells with the distance to this wall
                        // doing this brutally simple, for big maps this is inefficient
                        if (blockedCells[y * gridWidth + x] == 1)
                        {
                            sdfCells[y * gridWidth + x] = 0;
                            int minX = x - 10;
                            int minY = y - 10;
                            int maxX = x + 10;
                            int maxY = y + 10;
                            if (minX < 0) minX = 0;
                            if (minY < 0) minY = 0;
                            if (maxX >= gridWidth) maxX = gridWidth - 1;
                            if (maxY >= gridHeight) maxY = gridHeight - 1;
                            // update surrounding cells up to max distance we want to consider
                            for (int j = minY; j <= maxY; j++)
                            {
                                for (int i = minX; i <= maxX; i++)
                                {
                                    int dx = x - i;
                                    int dy = y - j;
                                    int d = 0;
                                    if (sdfFunction == 0)
                                    {
                                        // euclidean distance
                                        d = isqrt[dx * dx + dy * dy];
                                    }
                                    else if (sdfFunction == 1)
                                    {
                                        // chebyshev distance
                                        d = (abs(dx) < abs(dy)) ? abs(dy) : abs(dx);
                                    }
                                    else if (sdfFunction == 2)
                                    {
                                        // manhattan distance
                                        d = abs(dx) + abs(dy);
                                    }

                                    if (d < sdfCells[j*gridWidth + i] && d < 10)
                                    {
                                        sdfCells[j*gridWidth + i] = d;
                                    }
                                }
                            }
                        }
                    }
                }

                // trigger path finding for both entities
                FindPath(mapRat, pathRatStartX, pathRatStartY, pathRatEndX, pathRatEndY, 1, 
                    &pathRatLength, pathRat, ratWallFactor, jumpingEnabled);
                FindPath(mapCat, pathCatStartX, pathCatStartY, pathCatEndX, pathCatEndY, 2, 
                    &pathCatLength, pathCat, 0, jumpingEnabled);
            }

            //----------------------------------------------------------------------------------
            // draw cell content of walls and sdf values
            //----------------------------------------------------------------------------------
            for (int y = 0; y < gridHeight; y++)
            {
                for (int x = 0; x < gridWidth; x++)
                {
                    // blocked cells black
                    if (blockedCells[y * gridWidth + x] == 1) 
                    {
                        DrawRectangle(x * cellSize, y * cellSize, cellSize, cellSize, BLACK);
                    }

                    // sdf values as transparent blue (the further away from wals, the brighter)
                    int sdf = sdfCells[y * gridWidth + x];
                    DrawRectangle(x*cellSize, y * cellSize, cellSize, cellSize, (Color){ 32, 32, 32, 230-sdf * 20});
                }
            }

            //----------------------------------------------------------------------------------
            // draw rat pathfinding score data for visualization
            //----------------------------------------------------------------------------------
            PathfindingNode* pathToDraw = NULL;
            switch (visualizeMode % 3)
            {
                case 1: // visualize rat map
                    pathToDraw = mapRat;
                    break;
                case 2: // visualize cat map
                    pathToDraw = mapCat;
                    break;
            }

            if (pathToDraw != NULL)
            {
                int scoreMax = 0;
                for (int i = 0; i<gridWidth*gridHeight; i++)
                {
                    if (pathToDraw[i].score > scoreMax)
                    {
                        scoreMax = pathToDraw[i].score;
                    }
                }
                for (int y = 0; y < gridHeight; y++)
                {
                    for (int x = 0; x < gridWidth; x++)
                    {
                        if (pathToDraw[y * gridWidth + x].score > 0)
                        {
                            int score = pathToDraw[y * gridWidth + x].score;
                            int c = score % 64 * 4;
                            DrawRectangle(x*cellSize, y * cellSize, cellSize, cellSize, (Color){ c, c, 0, 128 });
                        }
                    }
                }
            }

            //----------------------------------------------------------------------------------
            // draw grid lines
            //----------------------------------------------------------------------------------
            for (int y = 0; y < gridHeight; y++)
            {
                DrawRectangle(0, y * cellSize, gridWidth*cellSize, 1, gridColor);
            }
            for (int x = 0; x < gridWidth; x++)
            {
                DrawRectangle(x*cellSize, 0, 1, gridHeight*cellSize, gridColor);
            }

            // highlight current cell the mouse is over
            DrawRectangle(cellX*cellSize, cellY * cellSize, cellSize, cellSize, cellHighlightColor);


            //----------------------------------------------------------------------------------
            // draw paths of cat and rat
            //----------------------------------------------------------------------------------
            for (int i = 0; i < pathCatLength; i++)
            {
                DrawRectangle(pathCat[i].x*cellSize + 1, pathCat[i].y * cellSize + 1, cellSize - 2, cellSize - 2, BLUE);
            }
            for (int i = 1; i < pathCatLength; i++)
            {
                DrawLine(pathCat[i-1].x*cellSize + cellSize / 2, pathCat[i-1].y * cellSize + cellSize / 2, pathCat[i].x*cellSize + cellSize / 2, pathCat[i].y * cellSize + cellSize / 2, BLUE);
            }

            for (int i = 0; i < pathRatLength; i++)
            {
                DrawRectangle(pathRat[i].x*cellSize + 4, pathRat[i].y * cellSize + 4, cellSize - 7, cellSize - 7, RED);
            }
            for (int i = 1; i < pathRatLength; i++)
            {
                DrawLine(pathRat[i-1].x*cellSize + cellSize / 2, pathRat[i-1].y * cellSize + cellSize / 2, pathRat[i].x*cellSize + cellSize / 2, pathRat[i].y * cellSize + cellSize / 2, RED);
            }

            //----------------------------------------------------------------------------------
            // draw animated movement of rat and cat
            //----------------------------------------------------------------------------------
            DrawPathMovement(pathRat, pathRatLength, &walkedPathDistanceRat, movementSpeed, cellSize * 0.5f + 2, (Color){128,0,0,255}, ratFace, sizeof(ratFace) / sizeof(ratFace[0]));
            DrawPathMovement(pathCat, pathCatLength, &walkedPathDistanceCat, movementSpeed, cellSize * 1.5f, (Color){0,0,128,255}, catFace, sizeof(catFace) / sizeof(catFace[0]));
            
            //----------------------------------------------------------------------------------
            // description and status
            //----------------------------------------------------------------------------------
            DrawText("Left click to toggle blocked cells, C: clear, Left mouse: toggle cell", 10, 10, 20, BLACK);
            DrawText("The red rat is small and likes to run close to walls", 10, 30, 20, RED);
            DrawText("The blue cat is big and can't fit through narrow paths and\nprefers the short path", 10, 50, 20, BLUE);
            char text[256];
            TextFormat(text, "Rat path length: %.2f, Cat path length: %.2f", CalcPathLength(pathRat, pathRatLength), CalcPathLength(pathCat, pathCatLength));
            DrawText(text, 10, GetScreenHeight() - 100, 20, BLACK);
            TextFormat(text, "R: randomize blocks, J: jumping enabled (current: %s)", jumpingEnabled ? "yes" : "no");
            DrawText(text, 10, GetScreenHeight() - 80, 20, BLACK);
            TextFormat(text, "S: switch SDF function (current: %s)", sdfFunction == 0 ? "euclidean" : (sdfFunction == 1 ? "chebyshev" : "manhattan"));
            DrawText(text, 10, GetScreenHeight() - 60, 20, BLACK);
            TextFormat(text, "Q: Rat wall factor (how much the rat wants to stay close to walls): %d", ratWallFactor);
            DrawText(text, 10, GetScreenHeight() - 40, 20, BLACK);
            TextFormat(text, "V: switch visualization mode (current: %s)", visualizeMode % 3 == 0 ? "none" : (visualizeMode % 3 == 1 ? "map rat" : "map cat"));
            DrawText(text, 10, GetScreenHeight() - 20, 20, BLACK);
        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    // De-Initialization
    //--------------------------------------------------------------------------------------
    CloseWindow();
    //--------------------------------------------------------------------------------------

    return 0;
}

