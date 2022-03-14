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