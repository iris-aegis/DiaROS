import Vue from 'vue';
import Router from 'vue-router';
import HelloWorld from './components/HelloWorld.vue';
import ChatLog from './components/chatlog.vue';
Vue.use(Router);
export default new Router({
    mode: 'history',
    base: process.env.BASE_URL,
    routes: [
        {
            path: '/',
            name: 'home',
            component: HelloWorld
        },
        {
            path: '/chatlog',
            name: 'chatlog',
            component: ChatLog
        }
    ]
});
//# sourceMappingURL=router.js.map