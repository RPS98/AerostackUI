
class MissionPlanner
{
    constructor() {
        this.addHTML();
        this.addCallbacks();
    }

    addHTML() {

        this.htmlId = 'sideBar-left-missionPlanner-content';

        let missionPlannerHtmlList = [];
        
        // Mission Dropdown list
        let missionList = ['New mission'];
        let missionListTotal = missionList.concat(MISSION_LIST.getMissionList());
        missionPlannerHtmlList.push(HTMLUtils.initDropDown(`${this.htmlId }-MissionList`, missionListTotal, 'New Mission'));

        // UAV Dropdown list
        missionPlannerHtmlList.push(HTMLUtils.initDropDown(`${this.htmlId }-UAVList`, UAV_LIST.getUavList(), UAV_LIST.getUavList()[0]));

        // Heigh input
        let heightInput = HTMLUtils.addDict('input', `${this.htmlId}-heightInput`, {'class': 'form-control', 'required': 'required',}, 'text', '1');
        let heightBtn   = HTMLUtils.addDict('button', `${this.htmlId}-heighBtn`, {'class': 'btn btn-primary'}, 'Set Height (m)');
        let heightRow  = HTMLUtils.addDict('splitDivs', 'none', {'class': 'row my-1 mx-1'}, [heightInput, heightBtn], {'class': 'col-md-6'});
        missionPlannerHtmlList.push(heightRow);


        // Buttons for change draw mode
        missionPlannerHtmlList.push(HTMLUtils.addDict('button', `${this.htmlId}-mouse`,  {'class': 'btn btn-primary m-1',}, `<i class="fas fa-mouse-pointer"></i>`));
        missionPlannerHtmlList.push(HTMLUtils.addDict('button', `${this.htmlId}-edit`,   {'class': 'btn btn-primary m-1',}, `<i class="fas fa-edit"></i>`));
        missionPlannerHtmlList.push(HTMLUtils.addDict('button', `${this.htmlId}-delete`, {'class': 'btn btn-primary m-1',}, `<i class="fas fa-eraser"></i>`));
        missionPlannerHtmlList.push(HTMLUtils.addDict('button', `${this.htmlId}-move`,   {'class': 'btn btn-primary m-1',}, `<i class="fas fa-arrows-alt"></i>`));
        missionPlannerHtmlList.push(HTMLUtils.addDict('button', `${this.htmlId}-rotate`, {'class': 'btn btn-primary m-1',}, `<i class="fas fa-sync-alt"></i>`));

        // Buttons for draw mission
        let splitBtn = [];
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-PoI`,     {'class': 'btn btn-primary',}, `Point of interest  <i class="fas fa-map-marker-alt"></i>`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-path`,    {'class': 'btn btn-primary',}, `Path <i class="fas fa-long-arrow-alt-up"></i>`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-area`,    {'class': 'btn btn-primary',}, `Area <i class="fas fa-draw-polygon"></i>`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-cArea`,   {'class': 'btn btn-primary',}, `Circular area <i class="far fa-circle"></i>`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-land`,    {'class': 'btn btn-primary',}, `Land point <i class="fas fa-h-square"></i>`));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-remove`,  {'class': 'btn btn-warning'},  'Remove all points'));
        splitBtn.push(HTMLUtils.addDict('button', `${this.htmlId}-confirm`, {'class': 'btn btn-success'},  'Confirm mission'));
        missionPlannerHtmlList.push(HTMLUtils.addDict('splitDivs', 'none', {}, splitBtn, {'class': 'row m-1'}));

        // Global collapse
        let missionPlannerCollapse = HTMLUtils.addDict('collapse', `${this.htmlId}-addToMapCollapse`, {}, 'Planner', true, missionPlannerHtmlList);
        HTMLUtils.addToExistingElement(`${this.htmlId}`, [missionPlannerCollapse]);
    }

    addCallbacks() {
        
    }
}
