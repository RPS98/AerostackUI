class UavInfo
{
    constructor() {
        M.UAV_MANAGER.addInfoAddCallback(this._newUavCallback.bind(this));
        M.UAV_MANAGER.addInfoChangeCallback(this._changeUavInfo.bind(this));
        this.uavSList = new SmartList();
        this.htmlId = 'sideBar-right-UAVInfo-content';

        this.header = config.SideBars.UavInfo.infoTableHeader;
        this.infoTable = config.SideBars.UavInfo.infoTable;
        this.roundTable = config.SideBars.UavInfo.roundTable;
    }

    _newUavCallback(myargs, args) {
        let uavId = args[0];

        ConsoleSideBar.addMessage(`UAV ${uavId} added`);

        if (this.uavSList.getList().indexOf(uavId) == -1) {
            this.uavSList.addObject(uavId, M.UAV_MANAGER.getDictById(uavId));
            this._addUav(uavId);
        }
    }

    _addUav(uavId) {

        let dict = this.uavSList.getDictById(uavId);
        let list = Utils.generateList(this.infoTable, dict, this.roundTable);
        
        let uavInfoHtml = HTMLUtils.addDict('table', `${this.htmlId}-UAV-${uavId}`, {}, this.header, list);
        let uavCollapseHtml = HTMLUtils.addDict('collapse', `${this.htmlId}-UAV-${uavId}`, {}, `${uavId}`, true, [uavInfoHtml]);
        HTMLUtils.addToExistingElement(`${this.htmlId}`, [uavCollapseHtml]);
        
    }

    _changeUavInfo(myargs, args) {
        let uavId = args[0];
        
        let parent = document.getElementById(`${this.htmlId}-UAV-${uavId}-table-body`);
        parent.innerHTML = '';

        let dict = this.uavSList.getDictById(uavId);
        let list = Utils.generateList(this.infoTable, dict, this.roundTable);

        for (let i = 0; i < list.length; i++) {
            let trContent = HTMLUtils.addDict('tr', ``, {}, list[i]);
            HTMLUtils.addHTML(parent, trContent);
        }
    }
}
