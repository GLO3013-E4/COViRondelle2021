import Main from "@/views/Main.vue";

export default {
  title: 'views/Main',
  component: Main,
};

export const Default = () => ({
  components: { Main },
  template: `<Main />`
});
