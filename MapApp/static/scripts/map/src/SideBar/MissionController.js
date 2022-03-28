class MissionController
{
    constructor()
    {
        this.htmlId = 'sideBar-left-missionController-content';
        this.initialized = false;
        M.MISSION_MANAGER.addInfoAddCallback(this.updateMissionListCallback.bind(this));

        this.initializedLoad();
    }

    initializedLoad() {
        HTMLUtils.addToExistingElement(`${this.htmlId}`, [HTMLUtils.addDict('fileInput', `${this.htmlId}-missionFile`, {}, 'Choose Mission File', '')]);
        Utils.addFileCallback(`${this.htmlId}-missionFile`, this.loadMissionCallback.bind(this));
        // HTMLUtils.addToExistingElement(`${this.htmlId}`, [HTMLUtils.addDict('button', `${this.htmlId}-LoadMission`, { 'class': 'btn btn-primary' }, 'Load Mission')]);
        // Utils.addButtonCallback(`${this.htmlId}-LoadMission`, this.loadMissionCallback.bind(this), []);
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
        Utils.addButtonCallback(`${this.htmlId}-btn-edit`,   this.editMissionCallback.bind(this), []);
        Utils.addButtonCallback(`${this.htmlId}-btn-save`,   this.saveMissionCallback.bind(this), []);
        Utils.addButtonCallback(`${this.htmlId}-btn-center`, this.centerMissionCallback.bind(this), []);
        Utils.addButtonCallback(`${this.htmlId}-btn-start`,  this.startMissionCallback.bind(this), []);
        Utils.addButtonCallback(`${this.htmlId}-btn-stop`,   this.stopMissionCallback.bind(this), []);
        Utils.addButtonCallback(`${this.htmlId}-btn-end`,    this.endMissionCallback.bind(this), []);
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

    editMissionCallback(args) {
        console.log(`Edit mission ${this.selectedMissionId}`);
    }

    centerMissionCallback(args) {
        console.log(`Center mission ${this.selectedMissionId}`);
    }

    stopMissionCallback(args) {
        console.log(`Stop mission ${this.selectedMissionId}`);
    }

    endMissionCallback(args) {
        console.log(`End mission ${this.selectedMissionId}`);
    }

    saveMissionCallback(myargs, args) {
        console.log('Save mission');

        let missionId = this.selectedMissionId;
        let missionDict = M.MISSION_MANAGER.getDictById(missionId);

        console.log(missionId);
        console.log(missionDict);

        Utils.download(`Mission-${missionId}.txt`, missionDict);
    }

    loadMissionCallback(args, input) {
        let fileToLoad = input.target.files[0];
        Utils.loadFile(fileToLoad, this.loadMissionFileCallback.bind(this), 'json', fileToLoad.name.split('.')[0]);
    }

    loadMissionFileCallback(data, myargs) {

        let input = document.getElementById(`${this.htmlId}-missionFile`);
        input.value = '';

        console.log("TODO: Load mission file");
        console.log(data)
        console.log(myargs)
        let id = myargs;
        let missionList = M.MISSION_MANAGER.getList();
        let cont = 0;
        while (missionList.includes(id)) {
            id = `${myargs}-${cont}`;
            cont++;
        }

        data.id = id;

        let uavList = M.UAV_MANAGER.getList();
        let missionUavList = data.uavList;

        for (let i = 0; i < missionUavList.length; i++) {
            let uavId = missionUavList[i];
            if (!uavList.includes(uavId)) {
                alert(`UAV ${uavId} from file ${myargs} is not connected`);
                return;
            }
        }

        M.WS.sendConfirmedMission(data);
    }
}