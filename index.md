---
layout: default
title: Home
---

<h1>Recent Posts</h1>

<div class="post-list">
  {% for post in site.posts limit:5 %}
  <div style="
    border: 1px solid #ddd;
    border-radius: 8px;
    padding: 1rem;
    margin-bottom: 1rem;
  ">
    <h2 style="margin-top: 0;">
      <a href="{{ post.url }}">{{ post.title }}</a>
    </h2>
    <small>{{ post.date | date: "%B %d, %Y" }}</small>

    {% if post.excerpt %}
    <p>{{ post.excerpt | strip_html }}</p>
    {% endif %}
  </div>
  {% endfor %}
</div>
