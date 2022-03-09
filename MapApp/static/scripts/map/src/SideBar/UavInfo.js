class UavInfo
{
    constructor() {
        M.UAV_MANAGER.addInfoAddCallback(this._newUavCallback.bind(this));
        M.UAV_MANAGER.addInfoChangeCallback(this._changeUavInfo.bind(this));
        this.uavSList = new SmartList();
        this.htmlId = 'sideBar-right-UAVInfo-content';
    }

    _newUavCallback(myargs, args) {
        let uavId = args[0];

        if (this.uavSList.getList().indexOf(uavId) == -1) {
            this.uavSList.addObject(uavId, M.UAV_MANAGER.getDictById(uavId));
            this._addUav(uavId);
        }
    }

    _addUav(uavId) {

        let header = ['Group', 'Type', 'Value'];
        let dict = this.uavSList.getDictById(uavId);
        
        let list = [
            ['Id', dict['id']],
            ['Info', 'Connected',       true], // dict['state']['connected']
            ['',     'Armed',           dict['state']['armed']],
            ['',     'Offboard',        dict['state']['offboard']],
            ['',     'State',           dict['state']['state']],
            ['',     'Yaw mode',        dict['state']['yaw_mode']],
            ['',     'Control mode',    dict['state']['control_mode']],
            ['',     'Reference frame', dict['state']['reference_frame']],
            ['Pose', 'lat',     Utils.round(dict['pose']['lat']   , 5)],
            ['',     'lng',     Utils.round(dict['pose']['lng']   , 5)],
            ['',      'height', Utils.round(dict['pose']['height'], 2)],
            ['',      'yaw',    Utils.round(dict['pose']['yaw']   , 2)],
        ];
        
        let uavInfoHtml = HTMLUtils.addDict('table', `${this.htmlId}-UAV-${uavId}`, {}, header, list);
        let uavCollapseHtml = HTMLUtils.addDict('collapse', `${this.htmlId}-UAV-${uavId}`, {}, `${uavId}`, true, [uavInfoHtml]);
        HTMLUtils.addToExistingElement(`${this.htmlId}`, [uavCollapseHtml]);
        
    }

    _changeUavInfo(myargs, args) {
        let uavId = args[0];
        
        let parent = document.getElementById(`${this.htmlId}-UAV-${uavId}-table-body`);
        parent.innerHTML = '';

        let dict = M.UAV_MANAGER.getDictById(uavId);

        let list = [
            ['Id', dict['id']],
            ['Info', 'Connected',       true], // dict['state']['connected']
            ['',     'Armed',           dict['state']['armed']],
            ['',     'Offboard',        dict['state']['offboard']],
            ['',     'State',           dict['state']['state']],
            ['',     'Yaw mode',        dict['state']['yaw_mode']],
            ['',     'Control mode',    dict['state']['control_mode']],
            ['',     'Reference frame', dict['state']['reference_frame']],
            ['Pose', 'lat',     Utils.round(dict['pose']['lat']   , 8)],
            ['',     'lng',     Utils.round(dict['pose']['lng']   , 8)],
            ['',      'height', Utils.round(dict['pose']['height'], 2)],
            ['',      'yaw',    Utils.round(dict['pose']['yaw']   , 2)],
        ];

        for (let i = 0; i < list.length; i++) {
            let trContent = HTMLUtils.addDict('tr', ``, {}, list[i]);
            HTMLUtils.addHTML(parent, trContent);
        }

        // let uavInfoHtml = HTMLUtils.addDict('table', `${this.htmlId}-UAV-${uavId}`, {}, header, list);
        // HTMLUtils.addToExistingElement(`${this.htmlId}-UAV-${uavId}-table-body`, [uavInfoHtml]);
    }
}