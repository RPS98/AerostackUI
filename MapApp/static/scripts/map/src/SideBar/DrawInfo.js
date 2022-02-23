class DrawInfo
{
    constructor() {
        M.UAV_MANAGER.addInfoAddCallback(this._newUavCallback.bind(this));
        this.uavSList = new SmartList();
        this.htmlId = 'sideBar-right-UAVInfo-content';
    }

    _newUavCallback(callback, uavId) {
        let uavList = M.UAV_MANAGER.getUavList();

        if (this.uavSList.indexOf(uavId) == -1) {
            this._addUav(uavId);
        }
    }

    _addUav(uavId) {
        let uavInfoHtml = HTMLUtils.addDict('div', `${this.htmlId}-UAV-${uavId}`, { 'class': 'btn btn-primary' }, 'Set Height (m)');
        HTMLUtils.addToExistingElement(`${this.htmlId}`, [uavInfoHtml]);
    }

    _addUavInfo(uavId, infoDict) {
        let parent = `${this.htmlId}-UAV-${uavId}`
    }
}