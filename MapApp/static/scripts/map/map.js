// TODO: Make this list generate dinaamically
/**
 * List of key and its corresponding class, to be used to generate HTML elements
 */
blocksClassList = [
    ['button', Button],
    ['checkBox', CheckBox],
    ['checkBoxes', CheckBoxes],
    ['collapse', Collapse],
    ['div', Div],
    ['dropDown', DropDown],
    ['dropDownBtn', DropDownBtn],
    ['dropDownExpand', DropDownExpand],
    ['input', Input],
    ['label', Label],
    ['splitDivs', SplitDivs],
    ['table', Table],
    ['tr', Tr],
]


window.onload = function () {

    // TODO: Load parameters from a config file
    // Map parameters
    let mapCenter = [28.14376, -16.50235];
    let mapZoom = 19;
    let connectionString = 'ws://127.0.0.1:8000/ws/user/';

    /**
     * Map Manager global variable to be accessed from other modules
     * @type {MapManager}
     * @global
     * @public
     */
    M = new MapManager(
        mapCenter,
        mapZoom,
        connectionString
    );

    M.initialize();

    /**
     * List of classes to be instantiated before the map is loaded
     */
    var sideBarsClass = [
        new Home(),
        new MissionPlanner(),
        new MissionController(),
        new UavDrawer(),
        new MissionDrawer(),
        new UavInfo(),
    ]

    // Instantiate sidebars elements
    for (let i = 0; i < sideBarsClass.length; i++) {
        sideBarsClass[i];
    }
}



// function generateNxNArray(n, m) {
//     var grid = [];
//     let iMax = n;
//     let jMax = m;
//     let count = 0;

//     for (let i = 0; i < iMax; i++) {
//         grid[i] = [];

//         for (let j = 0; j < jMax; j++) {
//             grid[i][j] = "";
//             count++;
//         }
//     }
//     return grid;
// } 


/*

function arrayToObject(array) {
    let obj = {};
    for (let i = 0; i < array.length; i++) {
        if (Array.isArray(array[i])) {
            obj[array[i][0]] = array[i][1];
        } else {
            obj[array[i]] = "";
        }
    }
    return obj;
}


function objectToArray(val) {
    //By default (val is not object or array, return the original value)
    var result = val;
    // If object passed the result is the return value of Object.entries()
    if (typeof val === 'object' && !Array.isArray(val)) {
        result = Object.entries(val);
        // If one of the results is an array or object, run this function on them again recursively
        result.forEach((attr) => {
            attr[1] = Tr.objectToArray(attr[1]);
        });
    }
    //If array passed, run this function on each value in it recursively
    else if (Array.isArray(val)) {
        val.forEach((v, i, a) => {
            a[i] = Tr.objectToArray(v)
        });
    }

    // Return the result
    return result;
}

function addTd(array) {
    for (let i = 0; i < array.length; i++) {
        if (!Array.isArray(array[i])) {
            let td = document.createElement('td');
            td.innerHTML = array[i];
            array[i] = td;
        } else if (Array.isArray(array[i])) {
            array[i] = addTd(array[i]);
        }
    }
    return array;
}



let dict = {
    'id': 'UAV 0',
    'state': 'landed',
    'pose': { 'lat': 28.144099, 'lng': -16.503337 },
}

*/