var aerostackUI = null;

/*
import MapManager
import WebSocketManager
import DrawManager
import UIManager
*/



/*
import home
import point
*/


window.onload = function()
{
    // TODO: Load parameters from a config file
    // Map parameters
    let map_center=[28.14376, -16.50235];
    let map_zoom=19;
    let connectionString = 'ws://127.0.0.1:8000/ws/user/';

    mapManager = new MapManager(
        connectionString,
        map_center,
        map_zoom
    );
};