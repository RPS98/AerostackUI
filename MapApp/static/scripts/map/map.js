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
    ['fileInput', FileInput],
]


window.onload = function () {

    // TODO: Load parameters from a config file
    // Map parameters
    let mapCenter = [28.14376, -16.50235];
    let mapZoom = 20;
    // let connectionString = 'ws://192.168.30.23:8000/ws/user/'; 
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
        new DrawInfo(),
        new MissionInfo(),
        new ImageOVerlay(),

    ]

    // Instantiate sidebars elements
    for (let i = 0; i < sideBarsClass.length; i++) {
        sideBarsClass[i];
    }

    // let marker = new DrawManager('Marker', 'test');
    // marker.codeDraw(mapCenter, {'rotationAngle': 0});
    // marker.codeLayerDrawn.setLatLng(mapCenter);
    // marker.codeLayerDrawn.remove();
}
