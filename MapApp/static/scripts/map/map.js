window.onload = function()
{
    // TODO: Load parameters from a config file
    // Map parameters
    let map_center=[28.14376, -16.50235];
    let map_zoom=19;
    let connectionString = 'ws://127.0.0.1:8000/ws/user/';

    mapManager = new MapManager(
        map_center,
        map_zoom
    );

    webSocketManager = new WebSocketManager(
        connectionString
    );

    //home = new Home();
    missionPlanner = new MissionPlanner();
};





// import {Utils, HTMLBlocks, HTMLDicts} from '/static/scripts/map/include/Utils/Utils.js';
// import {MapManager, UAV, Mission} from '/static/scripts/map/include/MapManager/MapManager.js';
// import {WebSocketManager} from '/static/scripts/map/include/WebSocketManager/WebSocketManager.js';


// /*
// import point
// */

// import {Home} from '/static/scripts/map/src/SideBar/LeftSideBar/Home/Home.js';



// var mapManager = new MapManager();
// var webSocketManager = new WebSocketManager();
// var drawManager = new DrawManager();
// var uiManager = new UIManager();



// mapManager.init();
// webSocketManager.init();
// drawManager.init();
// uiManager.init();



/*
function updateA(value) {
    A += value;
}

var A = 0;

updateA(2);

console.log(A);
*/

/*
// Not working

// file1.js
function updateA(value) {
    A += value;
}

// file2.js
var A = 0;

updateA(2);

console.log(A);

*/

/*
// Working

// file0.js
var A = 0;

// file1.js
function updateA(value) {
    A += value;
}

// file2.js
updateA(2);

console.log(A);

*/
