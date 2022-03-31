class MissionInfo
{
    constructor() {
        this.htmlId = 'sideBar-right-missionInfo-content';
        this.layerList = new SmartList();

        M.MISSION_MANAGER.addInfoAddCallback(this.addLayerCallback.bind(this));
        M.MISSION_MANAGER.addInfoChangeCallback(this.updateLayerCallback.bind(this));


        this.header = config.SideBars.MissionInfo.infoTableHeader;
        this.infoTable = config.SideBars.MissionInfo.infoTable;
        this.roundTable = config.SideBars.MissionInfo.roundTable;
    }    

    addLayerCallback(myargs, args) {
        
        let missionId = args[0];

        let dict = M.MISSION_MANAGER.getDictById(missionId);
        let list = Utils.generateList(this.infoTable, dict, this.roundTable);

        
        let missionInfoHtml = HTMLUtils.addDict('table', `${this.htmlId}-mission-${missionId}`, {}, this.header, list);
        let missionCollapseHtml = HTMLUtils.addDict('collapse', `${this.htmlId}-UAV-${missionId}`, {}, `Mission ${missionId}`, true, [missionInfoHtml]);
        HTMLUtils.addToExistingElement(`${this.htmlId}`, [missionCollapseHtml]);

    }

    updateLayerCallback(myargs, args) {

        let missionId = args[0];
        
        let parent = document.getElementById(`${this.htmlId}-mission-${missionId}-table-body`);
        parent.innerHTML = '';

        let dict = M.MISSION_MANAGER.getDictById(missionId);
        let list = Utils.generateList(this.infoTable, dict, this.roundTable);

        for (let i = 0; i < list.length; i++) {
            let trContent = HTMLUtils.addDict('tr', ``, {}, list[i]);
            HTMLUtils.addHTML(parent, trContent);
        }
    }
}