class DrawManager
{
    constructor () {

    }

    static codeDraw() {
        throw new Error("Method not implemented.");
    }

    static userDraw() {
        throw new Error("Method not implemented.");
    }

    static showInfo() {
        throw new Error("Method not implemented.");
    }

    static sendInfo() {
        throw new Error("Method not implemented.");
    }

    codeDraw(type) {
        
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