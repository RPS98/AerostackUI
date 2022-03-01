class MissionController
{
    constructor()
    {
        this.htmlId = 'sideBar-left-missionController-content';
        this.initialized = false;
        M.MISSION_MANAGER.addInfoAddCallback(this.updateMissionListCallback.bind(this));
    }

    addHTML() {
        this.selectedMissionId = M.MISSION_MANAGER.getList()[0];

        let missionControllerHtmlList = [];

        // Mission Dropdown list
        missionControllerHtmlList.push(HTMLUtils.initDropDown(`${this.htmlId }-MissionList`, M.MISSION_MANAGER.getList(), M.MISSION_MANAGER.getList()[0]));

        // Buttons for draw mission
        let splitBtn = [];
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-btn-reload`, {'class': 'btn btn-primary',}, `Reload missions`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-btn-edit`,   {'class': 'btn btn-primary',}, `Edit mission`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-btn-save`,   {'class': 'btn btn-primary',}, `Save mission`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-btn-center`, {'class': 'btn btn-primary',}, `Center mission`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-btn-start`,  {'class': 'btn btn-primary',}, `Start mission`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-btn-stop`,   {'class': 'btn btn-danger'},   'Stop mission'));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-btn-end`,    {'class': 'btn btn-success'},  'End mission'));
        missionControllerHtmlList.push(HTMLUtils.addDict('splitDivs', 'none', {}, splitBtn, {'class': 'row m-1'}));

        // Global collapse
        let missionControllerCollapse = HTMLUtils.addDict('collapse', `${this.htmlId}-addToMapCollapse`, {}, 'Controller', true, missionControllerHtmlList);
        HTMLUtils.addToExistingElement(`${this.htmlId}`, [missionControllerCollapse]);

        this.addControllerCallbacks();
    }

    addControllerCallbacks() {
        Utils.addButtonCallback(`${this.htmlId}-btn-start`, this.startMissionCallback.bind(this), []);
    }

    _checkInitalize() {
        if (!this.initialized) {
            this.addHTML();
            this.initialized = true;
        }
    }

    updateMissionListCallback(myargs, args) {
        this._checkInitalize();
        HTMLUtils.updateDropDown(`${this.htmlId}-MissionList`, M.MISSION_MANAGER.getList());
        this.addDropDownMissionCallback();
        this.selectedMissionId = M.MISSION_MANAGER.getList()[0];
    }

    clickMissionListCallback(e, args) {
        let button = document.getElementById(`${this.htmlId}-MissionList-DropDown-Btn`);
        button.innerHTML = e.innerHTML;
        this.selectedMissionId = e.innerHTML;
    }

    addDropDownMissionCallback() {
        Utils.addButtonsCallback(`${this.htmlId }-MissionList-item`, this.clickMissionListCallback.bind(this));
    }

    startMissionCallback(args) {
        console.log(`Start mission ${this.selectedMissionId}`);
        M.WS.sendStartMissionConfirm(this.selectedMissionId);
    }
}