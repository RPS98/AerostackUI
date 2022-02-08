
class MissionPlanner
{
    constructor() {
        this.addHTML();
        this.addCallbacks();
    }

    addHTML() {
        /*
        let btnAttributes = {
            'class': 'btn btn-primary',
        }

        let missionPlannerHtmlList = [];

        missionPlannerHtmlList.push(HTMLUtils.addDict('button', `Point of interest  <i class="fas fa-map-marker-alt"></i>`, 'sideBar-left-missionPlanner-content-PoI', btnAttributes));
       
        console.log("missionPlannerHtmlList");
        console.log(missionPlannerHtmlList);


        */
        let missionPlannerHtmlList = [];

        // Go to

        let btnAttributes = {
            'class': 'btn btn-primary',
        }
        missionPlannerHtmlList.push(HTMLUtils.addDict('button', 'sideBar-left-missionPlanner-content-PoI',   btnAttributes, `Point of interest  <i class="fas fa-map-marker-alt"></i>`));
        missionPlannerHtmlList.push(HTMLUtils.addDict('button', 'sideBar-left-missionPlanner-content-path',  btnAttributes, `Path <i class="fas fa-long-arrow-alt-up"></i>`));
        missionPlannerHtmlList.push(HTMLUtils.addDict('button', 'sideBar-left-missionPlanner-content-area',  btnAttributes, `Area <i class="fas fa-draw-polygon"></i>`));
        missionPlannerHtmlList.push(HTMLUtils.addDict('button', 'sideBar-left-missionPlanner-content-cArea', btnAttributes, `Circular area <i class="far fa-circle"></i>`));
        missionPlannerHtmlList.push(HTMLUtils.addDict('button', 'sideBar-left-missionPlanner-content-land',  btnAttributes, `Land point <i class="fas fa-h-square"></i>`));
        missionPlannerHtmlList.push(HTMLUtils.addDict('button', 'sideBar-left-missionPlanner-content-remove',  {'class': 'btn btn-warning'}, 'Remove all points'));
        missionPlannerHtmlList.push(HTMLUtils.addDict('button', 'sideBar-left-missionPlanner-content-confirm', {'class': 'btn btn-success'}, 'Confirm mission'));


        let missionPlannerCollapse = HTMLUtils.addDict('collapse', 'sideBar-left-missionPlanner-content-addToMapCollapse', {}, 'Add to map', missionPlannerHtmlList);

        HTMLUtils.addToExistingElement('sideBar-left-missionPlanner-content', [missionPlannerCollapse]);

        
    }

    addCallbacks() {

    }

}