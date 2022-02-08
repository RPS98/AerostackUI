
class MissionPlanner
{
    constructor() {
        this.addHTML();
        this.addCallbacks();
    }

    addHTML() {
        let missionPlannerHtmlList = [];

        let missionPlannerDropDownList = [];

        let dropDownBtn = HTMLUtils.addDict('button', 'sideBar-left-missionPlanner-content-DropDown-Btn', {'class': 'btn btn-info'}, 'New Mission');
        let dropDownExpandBtn = HTMLUtils.addDict('button', 'sideBar-left-missionPlanner-content-DropDown-Div-Btn', {'class': 'btn btn-secondary dropdown-MPMission-item w-75', 'style': "background: #dae8fc"}, 'New Mission');
        let dropDownExpand = HTMLUtils.addDict('ul', 'none', {}, [dropDownExpandBtn]);

        missionPlannerHtmlList.push(HTMLUtils.addDict('dropDown', 'sideBar-left-missionPlanner-content-missionPlannerDropDown', {'class': 'row m-1 gap-2'}, 'New Mission', dropDownBtn, dropDownExpand));

        // let heightInput = HTMLUtils.addDict('input', 'sideBar-left-missionPlanner-content-heightInput', {'class': 'form-number', 'required': 'required',}, 'number', '1');
        let heightInput = HTMLUtils.addDict('input', 'sideBar-left-missionPlanner-content-heightInput', {'class': 'form-control', 'required': 'required',}, 'text', '1');
        let heightBtn   = HTMLUtils.addDict('button', 'sideBar-left-missionPlanner-content-heighBtn', {'class': 'btn btn-primary'}, 'Set Height (m)');
        let heightRow  = HTMLUtils.addDict('splitDivs', 'none', {'class': 'row my-1 mx-1'}, [heightInput, heightBtn], {'class': 'col-md-6'});

        missionPlannerHtmlList.push(heightRow);

        missionPlannerHtmlList.push(HTMLUtils.addDict('button', 'sideBar-left-missionPlanner-content-mouse',  {'class': 'btn btn-primary m-1',}, `<i class="fas fa-mouse-pointer"></i>`));
        missionPlannerHtmlList.push(HTMLUtils.addDict('button', 'sideBar-left-missionPlanner-content-edit',   {'class': 'btn btn-primary m-1',}, `<i class="fas fa-edit"></i>`));
        missionPlannerHtmlList.push(HTMLUtils.addDict('button', 'sideBar-left-missionPlanner-content-delete', {'class': 'btn btn-primary m-1',}, `<i class="fas fa-eraser"></i>`));
        missionPlannerHtmlList.push(HTMLUtils.addDict('button', 'sideBar-left-missionPlanner-content-move',   {'class': 'btn btn-primary m-1',}, `<i class="fas fa-arrows-alt"></i>`));
        missionPlannerHtmlList.push(HTMLUtils.addDict('button', 'sideBar-left-missionPlanner-content-rotate', {'class': 'btn btn-primary m-1',}, `<i class="fas fa-sync-alt"></i>`));

        let splitBtn = [];
        splitBtn.push(HTMLUtils.addDict('button', 'sideBar-left-missionPlanner-content-PoI',     {'class': 'btn btn-primary',}, `Point of interest  <i class="fas fa-map-marker-alt"></i>`));
        splitBtn.push(HTMLUtils.addDict('button', 'sideBar-left-missionPlanner-content-path',    {'class': 'btn btn-primary',}, `Path <i class="fas fa-long-arrow-alt-up"></i>`));
        splitBtn.push(HTMLUtils.addDict('button', 'sideBar-left-missionPlanner-content-area',    {'class': 'btn btn-primary',}, `Area <i class="fas fa-draw-polygon"></i>`));
        splitBtn.push(HTMLUtils.addDict('button', 'sideBar-left-missionPlanner-content-cArea',   {'class': 'btn btn-primary',}, `Circular area <i class="far fa-circle"></i>`));
        splitBtn.push(HTMLUtils.addDict('button', 'sideBar-left-missionPlanner-content-land',    {'class': 'btn btn-primary',}, `Land point <i class="fas fa-h-square"></i>`));
        splitBtn.push(HTMLUtils.addDict('button', 'sideBar-left-missionPlanner-content-remove',  {'class': 'btn btn-warning'},  'Remove all points'));
        splitBtn.push(HTMLUtils.addDict('button', 'sideBar-left-missionPlanner-content-confirm', {'class': 'btn btn-success'},  'Confirm mission'));

        missionPlannerHtmlList.push(HTMLUtils.addDict('splitDivs', 'none', {}, splitBtn, {'class': 'row m-1'}));

        let missionPlannerCollapse = HTMLUtils.addDict('collapse', 'sideBar-left-missionPlanner-content-addToMapCollapse', {}, 'Add to map', true, missionPlannerHtmlList);

        HTMLUtils.addToExistingElement('sideBar-left-missionPlanner-content', [missionPlannerCollapse]);        
    }

    addCallbacks() {

    }

}