let cont = 0;
class ConsoleSideBar
{
    static addMessage(message) {
        let time = new Date();

        let htmlId = 'sideBar-left-console-content';
        let console = document.getElementById(htmlId);
        let newMessage = document.createElement('p');
        newMessage.innerHTML = cont + ' - ' + time.toLocaleTimeString() + ': ' + message;
        newMessage.style.color = 'white';
        newMessage.setAttribute('class', 'my-1');
        console.appendChild(newMessage);
        cont++;
    }

    static addWarning(message) {
        let time = new Date();
        let htmlId = 'sideBar-left-console-content';
        let console = document.getElementById(htmlId);
        let newMessage = document.createElement('p');
        newMessage.innerHTML = cont + ' - ' + time.toLocaleTimeString() + ': ' + message;
        newMessage.style.color = 'yellow';
        newMessage.setAttribute('class', 'my-1');
        console.appendChild(newMessage);
        cont++;
    }

    static addError(message) {
        let time = new Date();
        let htmlId = 'sideBar-left-console-content';
        let console = document.getElementById(htmlId);
        let newMessage = document.createElement('p');
        newMessage.innerHTML = cont + ' - ' + time.toLocaleTimeString() + ': ' + message;
        newMessage.style.color = 'red';
        newMessage.setAttribute('class', 'my-1');
        console.appendChild(newMessage);
        cont++;
    }
}