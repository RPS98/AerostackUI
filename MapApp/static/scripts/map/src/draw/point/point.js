class Point
{
    constructor() {
        mapManager.addButtonCallback(id, callbackFunction);

    }

    static codeDrawPoint(point, options={}) {
        drawManager.codeDraw(L.marker(point, options));
    }

    static userDrawPoint(point, options={}) {

        let custom_icon = iApp.gv['markers_icon_list'][iApp.gv['draw_selected_UAV']]
        options['markerStyle'] = {icon: custom_icon};

        drawManager.userDraw(L.marker(point, options));
    }

    static showPointInfo(id, layer_info, point) {
        let div = document.createElement("DIV");
    
        div.innerHTML = `
        <div class="d-grid gap-2 m-2">
            <button class="btn btn-secondary btn-collap-toggle" type="button" data-bs-toggle="collapse" data-bs-target="#collapseInfo${id}">
                Waypoint ${id}, ${layer_info['status']}, ${layer_info['UAVid']}, ${layer_info['height']} m
            <i class="icon-collap-toggle fas fa-plus"></i>
            </button>
            <div class="collapse multi-collapse show" id="collapseInfo${id}">
                <form id="info-form-${id}">
                    <div class="row my-1 mx-1">
                        <div class="col-6">
                            <input type="text" class="form-control" placeholder="latitude" required="required" value="${point.lat}" name="latitude">
                        </div>
                        <div class="col-6">
                            <input type="text" class="form-control" placeholder="longitude" required="required" value="${point.lng}" name="longitude">
                        </div>
                    </div>
                </form>
            </div>
        </div>
        `;
        div.setAttribute("id",`id-info-${id}`);
        return div
    }

    static sendInfo(id, point) {

        let msg = {};

        WebSocket.sendInfo(msg);

        // OR

        drawManager.sendInfo(msg);
    }
}