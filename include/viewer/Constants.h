#pragma once

// Screen settings
constexpr unsigned int SCR_WIDTH = 800;
constexpr unsigned int SCR_HEIGHT = 600;

// Grid dimensions
constexpr unsigned int GRID_WIDTH = 100;
constexpr unsigned int GRID_HEIGHT = 100;
constexpr float MAP_VIEW_PADDING_CELLS = 2.0f;
constexpr float GRID_LINE_MIN_PIXEL_SIZE = 3.0f;
constexpr float PATH_LINE_WIDTH = 4.0f;

// Agent settings
constexpr float DEFAULT_AGENT_DIAMETER_METERS = 2.0f;
constexpr float DEFAULT_AGENT_OBSERVATION_RADIUS_METERS = 5.0f;

// Camera settings
constexpr float DEFAULT_FOV = 90.0f;
constexpr float NEAR_DIST = 0.1f;
constexpr float FAR_DIST = 1000.0f;
constexpr float KEYBOARD_PAN_SPEED = 0.5f;