// class MissionInfo
// {
//     constructor() {
//         this.htmlId = 'sideBar-right-missionInfo-content';
//         this.layerList = new SmartList();
//         // M.addMapCallback('pm:create', this.createLayer.bind(this));

//         M.MISSION_MANAGER.addInfoAddCallback(this.addLayerCallback.bind(this));
//         M.MISSION_MANAGER.addInfoChangeCallback(this.updateLayerCallback.bind(this));
//     }    

//     addLayerCallback(myargs, args) {

//         if (args[1] == 'add') {
//             let info = M.DRAW_LAYERS.getDictById(args[0]);
//             info.drawManager.instance.addDrawInfo(this.htmlId, info);
//         } 
//     }

//     updateLayerCallback(myargs, args) {
//         let info = M.DRAW_LAYERS.getDictById(args[0]);
//         if (info != null) {
//             info.drawManager.instance.addDrawInfo(this.htmlId, info);
//         }
//     }
// }


class MissionInfo
{
    constructor() {
        this.htmlId = 'sideBar-right-missionInfo-content';
        this.layerList = new SmartList();
        // M.addMapCallback('pm:create', this.createLayer.bind(this));

        M.MISSION_MANAGER.addInfoAddCallback(this.addLayerCallback.bind(this));
        M.MISSION_MANAGER.addInfoChangeCallback(this.updateLayerCallback.bind(this));
    }    

    addLayerCallback(myargs, args) {
        console.log("MissionInfo: addLayerCallback");
        console.log(myargs);
        console.log(args);

        console.log(M.MISSION_MANAGER.getDictById(args[0]));
        

        let header = ['Group', 'Type', 'Value'];
        let dict = M.MISSION_MANAGER.getDictById(args[0]);

        let missionId = dict['id'];

        let layers = [];
        for (let i = 0; i < dict['layers'].length; i++) {
            layers.push(dict['layers'][i].name)
        }

        let list = [
            ['Id', dict['id']],
            ['Status', dict['status']],
            //['Uav List', dict['uavList']],
            //['Layers', layers],
        ];

        
        let missionInfoHtml = HTMLUtils.addDict('table', `${this.htmlId}-mission-${missionId}`, {}, header, list);
        let missionCollapseHtml = HTMLUtils.addDict('collapse', `${this.htmlId}-UAV-${missionId}`, {}, `Mission ${missionId}`, true, [missionInfoHtml]);
        HTMLUtils.addToExistingElement(`${this.htmlId}`, [missionCollapseHtml]);

        // if (args[1] == 'add') {
        //     let info = M.DRAW_LAYERS.getDictById(args[0]);
        //     info.drawManager.instance.addDrawInfo(this.htmlId, info);
        // } else if (args[1] == 'remove') {
        //     console.log("MissionInfo: removeCallback");
        //     console.log(args[0]);
        //     let info = M.DRAW_LAYERS.getDictById(args[0]);
        //     console.log(info)
        //     if (info != null) {
        //         info.drawManager.instance.removeDrawInfo(this.htmlId + '-' + info.id);
        //     }
        // }
    }

    updateLayerCallback(myargs, args) {
        console.log("MissionInfo: updateLayerCallback");
        console.log(myargs);
        console.log(args);
        console.log(M.MISSION_MANAGER.getDictById(args[0]));

        let missionId = dict['id'];
        
        let parent = document.getElementById(`${this.htmlId}-mission-${missionId}-table-body`);
        parent.innerHTML = '';

        let dict = M.MISSION_MANAGER.getDictById(args[0]);

        let layers = [];
        for (let i = 0; i < dict['layers'].length; i++) {
            layers.push(dict['layers'][i].name)
        }

        let list = [
            ['Id', dict['id']],
            ['Status', dict['status']],
            //['Uav List', dict['uavList']],
            //['Layers', layers],
        ];

        for (let i = 0; i < list.length; i++) {
            let trContent = HTMLUtils.addDict('tr', ``, {}, list[i]);
            HTMLUtils.addHTML(parent, trContent);
        }


        // let info = M.DRAW_LAYERS.getDictById(args[0]);
        // if (info != null) {
        //     info.drawManager.instance.addDrawInfo(this.htmlId, info);
        // }
    }
}