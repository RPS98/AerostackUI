class MissionController
{
    constructor()
    {
        this.htmlId = 'sideBar-left-missionController-content';

        let missionControllerHtmlList = [];

        // Mission Dropdown list
        missionControllerHtmlList.push(HTMLUtils.initDropDown(`${this.htmlId }-MissionList`, M.MISSION_LIST.getMissionList(), M.MISSION_LIST.getMissionList()[0]));

        // Buttons for draw mission
        let splitBtn = [];
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-btn-reload`, {'class': 'btn btn-primary',}, `Reload missions`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-btn-reload`, {'class': 'btn btn-primary',}, `Edit mission`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-btn-reload`, {'class': 'btn btn-primary',}, `Save mission`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-btn-reload`, {'class': 'btn btn-primary',}, `Center mission`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-btn-reload`, {'class': 'btn btn-primary',}, `Start mission`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-btn-reload`, {'class': 'btn btn-danger'},   'Stop mission'));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-btn-reload`, {'class': 'btn btn-success'},  'End mission'));
        missionControllerHtmlList.push(HTMLUtils.addDict('splitDivs', 'none', {}, splitBtn, {'class': 'row m-1'}));

        // Global collapse
        let missionControllerCollapse = HTMLUtils.addDict('collapse', `${this.htmlId}-addToMapCollapse`, {}, 'Controller', true, missionControllerHtmlList);
        HTMLUtils.addToExistingElement(`${this.htmlId}`, [missionControllerCollapse]);
    }
}