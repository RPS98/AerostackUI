class drawManager
{
    constructor () {

    }

    codeDraw(type) {
        type.addTo(mapManager.map);
        return type;
    }

    userDraw(type, options={}) {
        options['status'] = 'draw';
        options['missionId'] = iApp.gv['draw_selected_mission']; 
        options['UAVId'] = iApp.gv['draw_selected_UAV'];
        options['height'] = iApp.gv['draw_selected_height'];
        options['type'] = 'UAVMarker';
        
        mapManager.pm.enableDraw(type, options);
    }
}