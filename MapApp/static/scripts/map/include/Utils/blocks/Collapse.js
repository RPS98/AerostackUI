class Collapse extends HTMLUtilsPrototype 
{
    static addDict(type, id='none', attributes={}, ...args){
        return super.setDict(
            type,
            id,
            attributes,
            {
                'id': id,
                'text': args[0],
                'show': args[1],
                'list': args[2],
            }
        );
    }  

    static addHTML(content) {

        let div = document.createElement('div');
        let btn = document.createElement('button');
        let div_collapse = document.createElement('div');

        btn.setAttribute('class', 'btn btn-secondary btn-collap-toggle');
        btn.setAttribute('type', 'button');
        btn.setAttribute('data-bs-toggle', 'collapse');
        btn.setAttribute('data-bs-target', '#collapsable' + content.id);
        btn.innerHTML = `${content.text} <i class="icon-collap-toggle fas fa-plus"></i>`;

        if (content.show) {
            div_collapse.setAttribute('class', 'collapse multi-collapse show');
        } else {
            div_collapse.setAttribute('class', 'collapse multi-collapse');
        }
        div_collapse.setAttribute('id', 'collapsable' + content.id);
        
        for (let i=0; i<content.list.length; i++) {
            HTMLUtils.addHTML(div_collapse, content.list[i]);
        }

        div.setAttribute('class', 'd-grid gap-2 m-2');
        div.appendChild(btn);
        div.appendChild(div_collapse);

        return div;
    }
}